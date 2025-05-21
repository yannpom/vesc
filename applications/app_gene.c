/*
    Copyright 2025 Yann Pomarede yann.pomarede@gmail.com
    This file is part of the VESC firmware.
    License: GNU General Public License v3
*/

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "general.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "util/digital_filter.h"
#include "buffer.h"


#define LOOP_PERIOD 0.002f // 500 Hz
#define LOOP_PERIOD_MS ((int)(1000*LOOP_PERIOD)) // 2 ms

#define CAN_PERIOD 0.01f // 100 Hz
#define CAN_PERIOD_MS ((int)(1000*CAN_PERIOD)) // 10 ms
#define CAN_LOOP_RATIO (CAN_PERIOD_MS/LOOP_PERIOD_MS) // 5

#define BAFANG_CADENCE_TO_ERPM_RATIO (10000.0f/60.0f)  // 10k ERPM at 60 RPM
#define BAFANG_CADENCE_TO_ERPM(cadence) ((cadence)*BAFANG_CADENCE_TO_ERPM_RATIO)
#define BAFANG_ERPM_TO_CADENCE(erpm) ((erpm)/BAFANG_CADENCE_TO_ERPM_RATIO)

#define CAN_PACKET_GENE 63

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void terminal(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Config
static volatile float rpm_on = 3.0f;
static volatile float rpm_off = 1.5f;
static volatile float rpm_max = 100.0f;
static volatile float rpm_mini = 30.0f;
static volatile float min_brake_current = 0.1f;
static volatile float max_brake_current = 20.0f;
static volatile float pid_p = 0.8f; // Current / RPM
static volatile float pid_i = 40.0f;
static volatile float pid_d = 0.035f;
static volatile float d_filter_coeff = 0.2f;
static volatile float inertia = 0.25f;
static volatile float squared_losses_coeff = 3.0f; // A for 60 RPM
static volatile float plot_freq = 10.0f; // 10 Hz
static volatile float plot_mode = 1;

// Live data
static float actual_rpm = 0.0f;
static float rpm_goal = 0.0f;
static float e_rpm = 0.0f;
static float e_rpm_prev = 0.0f;
static float cmd_current = 0.0f;
static bool power_on = false;
static float p_term = 0.0f;
static float i_term = 0.0f;
static float d_term = 0.0f;
static float d_term_filtered = 0.0f;

static float current_bus = 0.0f;
static float voltage_bus = 0.0f;
static float watt = 0.0f;
static float watt_filtered = 0.0f;
static Biquad watt_filter1 = {};
static Biquad watt_filter2 = {};

static Biquad rear_current_filter = {};

static volatile float motors_rpm = 0; // 8700 = 39km/h

static volatile int loop_n[10] = {};

typedef struct {
    char * name;
    volatile float * value;
    char * description;
} param_t;

static param_t PARAMETERS[] = {
    {"Kp", &pid_p, "Proportional gain"},
    {"Ki", &pid_i, "Integral gain"},
    {"Kd", &pid_d, "Derivative gain"},
    {"df", &d_filter_coeff, "Derivative filter coefficient"},
    {"Cmin", &min_brake_current, "Minimum current (A)"},
    {"Cmax", &max_brake_current, "Maximum current (A)"},
    {"ron", &rpm_on, "RPM to turn on"},
    {"roff", &rpm_off, "RPM to turn off"},
    {"rmax", &rpm_max, "Maximum RPM"},
    {"rmini", &rpm_mini, "Minimum RPM goal"},
    {"inertia", &inertia, "Inertia"},
    {"loss", &squared_losses_coeff, "Squared losses coefficient (Current for 60 RPM)"},
    {"plot_freq", &plot_freq, "Plot frequency"},
    {"plot_mode", &plot_mode, "0=off, 1=auto, 2=on"}
};



// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
    stop_now = false;
    chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa), NORMALPRIO, my_thread, NULL);

    // Terminal commands for the VESC Tool terminal can be registered.
    terminal_register_command_callback("gene", "Print the Gene parameters", 0, terminal);

    biquad_config(&watt_filter1, BQ_LOWPASS, LOOP_PERIOD*5.0f);
    biquad_config(&watt_filter2, BQ_LOWPASS, LOOP_PERIOD*0.25f);
    biquad_config(&rear_current_filter, BQ_LOWPASS, LOOP_PERIOD*5.0f);
    
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
    mc_interface_set_pwm_callback(0);
    terminal_unregister_callback(terminal);

    stop_now = true;
    while (is_running) {
        chThdSleepMilliseconds(1);
    }
}

void app_custom_configure(app_configuration *conf) {
    (void)conf;
}

static THD_FUNCTION(my_thread, arg) {
    (void)arg;

    chRegSetThreadName("APP_GENE");

    is_running = true;

    // Experiment plot
    commands_init_plot("X", "Y");
    commands_plot_add_graph("RPM goal");
    commands_plot_add_graph("RPM");
    commands_plot_add_graph("Current");
    commands_plot_add_graph("Watt");
    commands_plot_add_graph("Watt filtered");
    // commands_plot_add_graph("P Term");
    // commands_plot_add_graph("I Term");
    // commands_plot_add_graph("D Term");

    systime_t next_time = chVTGetSystemTimeX();  // Get current system time

    const app_configuration *conf = app_get_configuration();
    const int can_id = conf->controller_id;

    while (true) {
        loop_n[1]++;
        // Check if it is time to stop.
        if (stop_now) {
            is_running = false;
            return;
        }
        timeout_reset(); // Reset timeout if everything is OK.

        loop_n[2]++;

        // RPM error
        actual_rpm = BAFANG_ERPM_TO_CADENCE(fabsf(mc_interface_get_rpm()));

        e_rpm = actual_rpm - rpm_goal;
        
        p_term = pid_p * e_rpm;

        d_term = pid_d * (e_rpm - e_rpm_prev) / LOOP_PERIOD;

        d_term_filtered += d_filter_coeff * (d_term - d_term_filtered);

        i_term += LOOP_PERIOD*(pid_i * e_rpm);
        utils_truncate_number(&i_term, 0, max_brake_current);

        e_rpm_prev = e_rpm;

        cmd_current = p_term + d_term_filtered + i_term;
        utils_truncate_number(&cmd_current, min_brake_current, max_brake_current);

        loop_n[3]++;

        if (actual_rpm > rpm_on) {
            power_on = true;
        } else if (power_on && actual_rpm < rpm_off) {
            power_on = false;
        }

        if (!power_on) {
            mc_interface_release_motor();
        }

        // Compute the torques
        float pedal_torque = cmd_current;
        float friction_torque = min_brake_current;
        float air_resistance_torque = squared_losses_coeff * (rpm_goal/60.0f) * (rpm_goal/60.0f);
        float net_torque = pedal_torque - friction_torque - air_resistance_torque;

        // Adjust the RPM goal
        if (can_id == 1) {
            float new_rpm_goal = motors_rpm / 8700.0f * 80.0f;
            if (new_rpm_goal < rpm_mini) {
                new_rpm_goal = rpm_mini;
            }
            // rpm_goal = fmaxf(new_rpm_goal, 4.0f);
            rpm_goal = 0.97f * rpm_goal + 0.03f * new_rpm_goal;
        } else {
            rpm_goal += net_torque * LOOP_PERIOD / inertia;
            if (rpm_goal < rpm_mini) {
                rpm_goal = rpm_mini;
            } else if (rpm_goal > rpm_max) {
                rpm_goal = rpm_max;
            }
        }

        loop_n[4]++;

        // Drive motor
        if (power_on) {
            mc_interface_set_brake_current(cmd_current);
        }

        loop_n[5]++;

        // Measure power
        current_bus = fabsf(mc_interface_get_tot_current_in_filtered());
        voltage_bus = mc_interface_get_input_voltage_filtered();
        watt = biquad_process(&watt_filter1, voltage_bus * current_bus);
        watt_filtered = biquad_process(&watt_filter2, watt);

        const float rear_current = fminf(120.0f, 12.0f*(cmd_current-min_brake_current));
        const float rear_current_filtered = biquad_process(&rear_current_filter, rear_current);


        loop_n[6]++;

        // Plot
        // if (power_on && (loop_n%10==0)) {

        const int modulo = 1.0f/(LOOP_PERIOD*plot_freq);

        bool plot_active = false;
        switch ((int)plot_mode)
        {
            case 1: plot_active = power_on; break;
            case 2: plot_active = true; break;
        }
        if (plot_active && (loop_n[0]%modulo==0)) {
            float x = loop_n[0] * LOOP_PERIOD;
            commands_plot_set_graph(0);
            commands_send_plot_points(x, rpm_goal);
            commands_plot_set_graph(1);
            commands_send_plot_points(x, actual_rpm);
            commands_plot_set_graph(2);
            commands_send_plot_points(x, cmd_current);
            commands_plot_set_graph(3);
            commands_send_plot_points(x, watt);
            commands_plot_set_graph(4);
            commands_send_plot_points(x, watt_filtered);
            
            // commands_plot_set_graph(0);
            // commands_send_plot_points(x, rpm_goal);
            // commands_plot_set_graph(1);
            // commands_send_plot_points(x, actual_rpm);
            // commands_plot_set_graph(2);
            // commands_send_plot_points(x, cmd_current);
            // commands_plot_set_graph(3);
            // commands_send_plot_points(x, p_term);
            // commands_plot_set_graph(4);
            // commands_send_plot_points(x, i_term);
            // commands_plot_set_graph(5);
            // commands_send_plot_points(x, d_term);
            
        }

        loop_n[7]++;
        // CAN
        if (power_on && (loop_n[0] % CAN_LOOP_RATIO == 0)) { // 100 Hz
            int32_t send_index = 0;
            uint8_t buffer[6];
            buffer_append_int16(buffer, (int16_t)(10.0f*rpm_goal), &send_index);
            buffer_append_int16(buffer, (int16_t)(10.0f*actual_rpm), &send_index);
            buffer_append_int16(buffer, (int16_t)(10.0f*watt_filtered), &send_index);
            comm_can_transmit_eid_replace(can_id | ((uint32_t)CAN_PACKET_GENE << 8), buffer, send_index, true, 0);

            // comm_can_set_current_off_delay(255, cmd_current, 1.0f);
            
            // comm_can_set_current(255, rear_current_filtered);   
        }

        can_status_msg * status1 = comm_can_get_status_msg_id(5);
        // can_status_msg * status2 = comm_can_get_status_msg_id(6);
        if (status1) {
            motors_rpm = status1->rpm;

        }

        if (can_id == 1) {
            if (rear_current_filtered>1.0f && (loop_n[0] % CAN_LOOP_RATIO == 0)) { // 100 Hz
                comm_can_set_current(5, rear_current_filtered);
                comm_can_set_current(6, rear_current_filtered);
            }
        }

        loop_n[8]++;

        // Sleep until the next scheduled time
        next_time += MS2ST(LOOP_PERIOD_MS);
        // chThdSleepUntil(next_time);
        chThdSleepMilliseconds(2);
        loop_n[0]++;
    }
}

// Callback function for the terminal command with arguments.
static void terminal(int argc, const char **argv) {
    if (argc == 1) {
        commands_printf("Gene parameters:");
        for (size_t i = 0; i < sizeof(PARAMETERS)/sizeof(param_t); i++) {
            commands_printf("  %s: %.3f", PARAMETERS[i].name, (double)*(PARAMETERS[i].value));
        }
        commands_printf("loop_n: %d %d %d %d %d %d %d %d %d %d", loop_n[0], loop_n[1], loop_n[2], loop_n[3], loop_n[4], loop_n[5], loop_n[6], loop_n[7], loop_n[8], loop_n[9]);
        commands_printf("is_running: %d", is_running);
        commands_printf("stop_now: %d", stop_now);
        commands_printf("motors_rpm: %f", (double)motors_rpm);
        // commands_printf("motors_id: %d", (double)id);
        
        
        
    } else if (argc == 3) {
        const char* param = argv[1];
        float value = atof(argv[2]);
        for (size_t i = 0; i < sizeof(PARAMETERS)/sizeof(param_t); i++) {
            if (strcasecmp(param, PARAMETERS[i].name) == 0) {
                float old_value = *(PARAMETERS[i].value);
                *(PARAMETERS[i].value) = value;
                commands_printf("Set %s from %.3f to %.3f", PARAMETERS[i].name, (double)old_value, (double)value);
                return;
            }
        }
        commands_printf("Invalid parameter");
    } else {
        commands_printf("Usage: gene [param] [value]");
    }
}
