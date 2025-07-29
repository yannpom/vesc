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

#define CAN_PERIOD 0.05f // 20 Hz
#define CAN_PERIOD_MS ((int)(1000*CAN_PERIOD)) // 50 ms


#define BAFANG_CADENCE_TO_ERPM_RATIO (10000.0f/60.0f)  // 10k ERPM at 60 RPM
#define BAFANG_CADENCE_TO_ERPM(cadence) ((cadence)*BAFANG_CADENCE_TO_ERPM_RATIO)
#define BAFANG_ERPM_TO_CADENCE(erpm) ((erpm)/BAFANG_CADENCE_TO_ERPM_RATIO)

#define CAN_PACKET_GENE 0x03

#define TRUNC_MIN_MAX(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))


#pragma pack(push, 1)
typedef struct {
    union {
        uint8_t value;
        struct {
            unsigned walk:1; // 0 = drive, 1 = walk
            unsigned brake:1; // 0 = drive, 1 = brake
        };
    };
} can_bitfield;
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct {
    unsigned power_on : 1;
    unsigned blinker_left : 1;
    unsigned blinker_right : 1;
    unsigned horn : 1;
    unsigned interior_light : 1;
    unsigned wiper : 1;
    unsigned brake_level: 2;
    unsigned walk_forward: 1;
    unsigned walk_backward: 1;
    unsigned padding : 6;
} state_can_t;
#pragma pack(pop)

state_can_t state_can = {};


// Threads
static THD_WORKING_AREA(main_thread_wa, 1024);
static THD_WORKING_AREA(can_thread_wa, 1024);

static THD_FUNCTION(gene_thread, arg);
static THD_FUNCTION(gene_can_thread, arg);
static THD_FUNCTION(motor_thread, arg);
static THD_FUNCTION(motor_can_thread, arg);


// Private functions
static void terminal_gene(int argc, const char **argv);
static void terminal_motor(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool thread_main_is_running = false;
static volatile bool thread_can_is_running = false;

// Config
static volatile float rpm_on = 3.0f;
static volatile float rpm_off = 1.5f;
static volatile float rpm_max = 100.0f;
static volatile float rpm_mini = 20.0f;
static volatile float min_brake_current = 0.1f;
static volatile float max_brake_current = 25.0f;
static volatile float current_gain = 18.00f; // A motor / A gene
static volatile float pid_p = 0.8f; // Current / RPM
static volatile float pid_i = 40.0f;
static volatile float pid_d = 0.035f;
static volatile float d_filter_coeff = 0.2f;
static volatile float inertia = 0.25f;
static volatile float squared_losses_coeff = 3.0f; // A for 60 RPM
static volatile float plot_freq = 10.0f; // 10 Hz
static volatile float plot_mode = 0;
static volatile float walk_speed_rpm = 600.0f;

static uint8_t can_id = 0;

// Live data
static volatile uint8_t mode = 0; // 0=neutral, 1=forward, 2=reverse, 3=walk
static volatile float actual_rpm = 0.0f;
static volatile float rpm_goal = 0.0f;
static volatile float e_rpm = 0.0f;
static volatile float e_rpm_prev = 0.0f;
static volatile float cmd_current = 0.0f;
static volatile bool power_on = false;
static volatile float p_term = 0.0f;
static volatile float i_term = 0.0f;
static volatile float d_term = 0.0f;
static volatile float d_term_filtered = 0.0f;
static volatile float rear_current = 0.0f;
static volatile float rear_current_filtered = 0.0f;

static volatile float current_bus = 0.0f;
static volatile float voltage_bus = 0.0f;
static volatile float watt = 0.0f;
static volatile float watt_filtered = 0.0f;
static Biquad watt_filter1 = {};
static Biquad watt_filter2 = {};

static Biquad rear_current_filter = {};

static volatile float motors_rpm[2] = {}; // 3900 = 25km/h
static volatile int gene_thread_loop_n = 0;


// Motor stuff
static volatile float motor_driving_current_goal = 0.0f; // Current from Gene input
static volatile int motor_driving_current_goal_age = 0; // age of the command
static volatile float motor_brake_current_goal = 0.0f; // Current from brakes input
static volatile float motor_driving_current = 0.0f; // Actual current goal (brake has precedence over gene)
static volatile float motor_braking_current = 0.0f;
static volatile float motor_walk_direction = 0.0f;

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
    {"gain", &current_gain, "Drive current gain (A/A)"},
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

    biquad_config(&watt_filter1, BQ_LOWPASS, LOOP_PERIOD * 5.0f);
    biquad_config(&watt_filter2, BQ_LOWPASS, LOOP_PERIOD * 1.0f);
    biquad_config(&rear_current_filter, BQ_LOWPASS, LOOP_PERIOD * 2.5f);

    const app_configuration *conf = app_get_configuration();
    can_id = conf->controller_id;

    if (can_id == 1 || can_id == 2 || can_id == 3 || can_id == 4) {
        // Gene
        chThdCreateStatic(main_thread_wa, sizeof(main_thread_wa), NORMALPRIO+10, gene_thread, NULL);
        chThdCreateStatic(can_thread_wa, sizeof(can_thread_wa), NORMALPRIO, gene_can_thread, NULL);
        terminal_register_command_callback("gene", "Print the Gene parameters", 0, terminal_gene);
    } else if (can_id == 5 || can_id == 6) {
        // Motors
        chThdCreateStatic(main_thread_wa, sizeof(main_thread_wa), NORMALPRIO+10, motor_thread, NULL);
        chThdCreateStatic(can_thread_wa, sizeof(can_thread_wa), NORMALPRIO, motor_can_thread, NULL);
        terminal_register_command_callback("motor", "Print the Motor parameters", 0, terminal_motor);
    }
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
    mc_interface_set_pwm_callback(0);
    terminal_unregister_callback(terminal_gene);
    terminal_unregister_callback(terminal_motor);

    stop_now = true;
    while (thread_main_is_running || thread_can_is_running) {
        chThdSleepMilliseconds(1);
    }
}

void app_custom_configure(app_configuration *conf) {
    (void)conf;
}

inline void limit_rate(float * value, float target, float max_rate) {
    float diff = target - *value;
    diff = TRUNC_MIN_MAX(diff, -max_rate, max_rate);
    *value += diff;
}


static THD_FUNCTION(motor_thread, arg) {
    (void)arg;

    chRegSetThreadName("APP_MOTOR");

    while (true) {
        if (stop_now) {
            return;
        }

        

        motor_driving_current_goal_age++;
        if (motor_driving_current_goal_age > 200) {
            motor_driving_current_goal = 0;
        }

    
        if (motor_brake_current_goal > 0) {
            if (fabsf(motor_driving_current) > 1.0f) {
                // we are driving, we need to decrease current before braking
                limit_rate(&motor_driving_current, 0, 1.0f);
                motor_braking_current = 0;
            } else {
                // driving current is null, start increase the brake current
                limit_rate(&motor_braking_current, motor_brake_current_goal, 1.0f);
                motor_driving_current = 0;
            }
        } else {
            // driving wanted
            if (motor_braking_current > 1.0f) {
                // but we are currenctly braking
                limit_rate(&motor_braking_current, 0, 1.0f);
                motor_driving_current = 0;
            } else {
                // no braking ongoing
                limit_rate(&motor_driving_current, motor_driving_current_goal, 1.0f);
                motor_braking_current = 0;
            }
        }


        if (mode == 1 || mode == 2) {
            if (motor_braking_current) {
                mc_interface_set_brake_current(motor_braking_current);
            } else {
                mc_interface_set_current(motor_driving_current);
            }
        } else if (mode == 3) {
            if (motor_walk_direction != 0.0f) {
                if (motor_walk_direction > 0) {
                    mc_interface_set_pid_speed(walk_speed_rpm);
                } else {
                    mc_interface_set_pid_speed(-walk_speed_rpm);
                }
            } else {
                mc_interface_set_current(0);
            }
        }        
        
        timeout_reset();
        chThdSleepMilliseconds(1);
    }
}

static THD_FUNCTION(gene_thread, arg) {
    (void)arg;

    chRegSetThreadName("APP_GENE");

    thread_main_is_running = true;

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

    while (true) {
        // Check if it is time to stop.
        if (stop_now) {
            thread_main_is_running = false;
            return;
        }
        timeout_reset(); // Reset timeout if everything is OK.

        // RPM error
        actual_rpm = BAFANG_ERPM_TO_CADENCE(fabsf(mc_interface_get_rpm()));

        e_rpm = actual_rpm - rpm_goal;
        
        p_term = pid_p * e_rpm;

        d_term = pid_d * (e_rpm - e_rpm_prev) / LOOP_PERIOD;

        d_term_filtered += d_filter_coeff * (d_term - d_term_filtered);

        i_term += LOOP_PERIOD*(pid_i * e_rpm);
        i_term = TRUNC_MIN_MAX(i_term, 0, max_brake_current);

        e_rpm_prev = e_rpm;

        cmd_current = p_term + d_term_filtered + i_term;
        cmd_current = TRUNC_MIN_MAX(cmd_current, min_brake_current, max_brake_current);

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
        if (can_id == 1 && mode > 0) {
            float new_rpm_goal = fmaxf(motors_rpm[0], motors_rpm[1]) / 3900.0f * 62.0f;
            if (new_rpm_goal < rpm_mini) {
                new_rpm_goal = rpm_mini;
            }
            rpm_goal = 0.97f * rpm_goal + 0.03f * new_rpm_goal;
        } else {
            rpm_goal += net_torque * LOOP_PERIOD / inertia;
            if (rpm_goal < rpm_mini) {
                rpm_goal = rpm_mini;
            } else if (rpm_goal > rpm_max) {
                rpm_goal = rpm_max;
            }
        }

        // Drive motor
        if (power_on) {
            mc_interface_set_brake_current(cmd_current);
        }

        // Measure power
        current_bus = fabsf(mc_interface_get_tot_current_in_filtered());
        voltage_bus = mc_interface_get_input_voltage_filtered();
        watt = biquad_process(&watt_filter1, voltage_bus * current_bus);
        watt_filtered = biquad_process(&watt_filter2, watt);

        rear_current = current_gain*(cmd_current-min_brake_current);
        rear_current_filtered = biquad_process(&rear_current_filter, rear_current);

        // TODO mode to another thread
        // Plot
        // const int modulo = 1.0f/(LOOP_PERIOD*plot_freq);
        // bool plot_active = false;
        // switch ((int)plot_mode)
        // {
        //     case 1: plot_active = power_on; break;
        //     case 2: plot_active = true; break;
        // }
        // if (plot_active && (loop_n[0]%modulo==0)) {
        //     float x = loop_n[0] * LOOP_PERIOD;
        //     commands_plot_set_graph(0);
        //     commands_send_plot_points(x, rpm_goal);
        //     commands_plot_set_graph(1);
        //     commands_send_plot_points(x, actual_rpm);
        //     commands_plot_set_graph(2);
        //     commands_send_plot_points(x, cmd_current);
        //     commands_plot_set_graph(3);
        //     commands_send_plot_points(x, watt);
        //     commands_plot_set_graph(4);
        //     commands_send_plot_points(x, watt_filtered);
        // }

        // Sleep until the next scheduled time
        do {
            next_time += MS2ST(LOOP_PERIOD_MS);
        } while (next_time <= chVTGetSystemTimeX());
        
        chThdSleepUntil(next_time);
        gene_thread_loop_n++;
    }
}


static bool can_sid_callback(uint32_t id, uint8_t *data, uint8_t len) {
    int32_t index = 0;
    if (id == 0x45 || id == 0x46) {
        // motor reporting
        /*int16_t voltage_in = */buffer_get_int16(data, &index);
        /*int16_t current_in = */buffer_get_int16(data, &index);
        int16_t rpm = buffer_get_int16(data, &index);
        /*int8_t current = */buffer_get_int8(data, &index);
        /*int8_t temperature = */buffer_get_int8(data, &index);

        motors_rpm[id-0x45] = rpm;

    } else if (id == 0x1b) {
        // Mode from screen
        mode = buffer_get_int8(data, &index);
    } else if (id == 0x31) {
        // Current Command from Gene1
        float current = buffer_get_int16(data, &index) / 100.0f;
        buffer_get_int16(data, &index);
        buffer_get_int16(data, &index);
        can_bitfield bitfield;
        bitfield.value = buffer_get_int8(data, &index);
        // bitfield.brake
        // bitfield.walk
        if (state_can.brake_level > 0) {
            motor_driving_current_goal = 0;
        } else {
            motor_driving_current_goal = current;
            motor_driving_current_goal_age = 0;
            motor_brake_current_goal = 0;
        }
        
    } else if (id == 0x27) { // outputs
        memcpy(&state_can, data, sizeof(state_can));
        if (state_can.walk_forward) {
            motor_walk_direction = 1.0f;
        } else if (state_can.walk_backward) {
            motor_walk_direction = -1.0f;
        } else {
            motor_walk_direction = 0.0f;
        }
        if (state_can.brake_level == 1) {
            motor_brake_current_goal = 50.0f;
            motor_driving_current_goal = 0;
        } else if (state_can.brake_level == 2) {
            motor_brake_current_goal = 120.0f;
            motor_driving_current_goal = 0;
        } else {
            motor_brake_current_goal = 0;
        }
    }
    return true;
}

static THD_FUNCTION(motor_can_thread, arg) {
    (void)arg;

    chRegSetThreadName("APP_MOTOR_CAN");

    thread_can_is_running = true;

    comm_can_set_sid_rx_callback(can_sid_callback);

    systime_t next_time = chVTGetSystemTimeX();  // Get current system time

    const int can_period_ms = 50;

    int i = 0;

    while (true) {
        // Check if it is time to stop.
        if (stop_now) {
            thread_can_is_running = false;
            return;
        }
    
        if (mc_interface_get_state() == MC_STATE_RUNNING || i%20==0) {
            

            float voltage_in = mc_interface_get_input_voltage_filtered();
            float current_in = mc_interface_get_tot_current_in_filtered();
            float rpm = mc_interface_get_rpm();
            float current = mc_interface_get_tot_current_filtered();
            float temperature_fet = mc_interface_temp_fet_filtered();
            float temperature_motor = mc_interface_temp_motor_filtered();

            static int n = 0;
            uint8_t temperature = 0;
            if (n++%2==0) {
                temperature = (uint8_t)temperature_fet;
            } else {
                temperature = (uint8_t)temperature_motor;
            }
            temperature = (temperature&0x7F) | ((n%2)<<7);

            int32_t send_index = 0;
            uint8_t buffer[8];
            buffer_append_int16(buffer, (int16_t)(100.0f*voltage_in), &send_index);
            buffer_append_int16(buffer, (int16_t)(100.0f*current_in), &send_index);
            buffer_append_int16(buffer, (int16_t)(rpm), &send_index);
            buffer_append_uint8(buffer, (uint8_t)(current), &send_index);
            buffer_append_uint8(buffer, (uint8_t)(temperature), &send_index);
            comm_can_transmit_sid(0x40 | can_id, buffer, send_index);
        }
        
        // Sleep until the next scheduled time
        do {
            next_time += MS2ST(can_period_ms);
        } while (next_time <= chVTGetSystemTimeX());
        
        chThdSleepUntil(next_time);
        i++;
    }
}


static THD_FUNCTION(gene_can_thread, arg) {
    (void)arg;

    chRegSetThreadName("APP_GENE_CAN");

    thread_can_is_running = true;

    comm_can_set_sid_rx_callback(can_sid_callback);

    systime_t next_time = chVTGetSystemTimeX();  // Get current system time

    const int can_period_ms = can_id == 1 ? CAN_PERIOD_MS : 1000;

    int i = 0;
    while (true) {
        // Check if it is time to stop.
        if (stop_now) {
            thread_can_is_running = false;
            return;
        }
    
        bool send_frame = false;
        if (can_id == 1) {
            if (power_on || rear_current_filtered>.01f) {
                send_frame = true;
            } else {
                // avoid flooding when power_off
                send_frame = i%20==0;
            }
        } else {
            send_frame = true;
        }
            
        
        if (send_frame) {
            int32_t send_index = 0;
            uint8_t buffer[8];

            float current_to_send = 0;
            switch (mode) {
                case 1: current_to_send = rear_current_filtered; break;
                case 2: current_to_send = -rear_current_filtered; break;
                case 3: current_to_send = 0; break;
            }
            current_to_send = TRUNC_MIN_MAX(current_to_send, -300.0f, 300.0f);
            buffer_append_int16(buffer, (int16_t)(100.0f*current_to_send), &send_index);
            buffer_append_int16(buffer, (int16_t)(10.0f*watt_filtered), &send_index);
            buffer_append_int16(buffer, (int16_t)(100.0f*actual_rpm), &send_index);
            can_bitfield bitfield = {};
            // bitfield.brake = 0;
            buffer_append_int8(buffer, bitfield.value, &send_index);
            comm_can_transmit_sid(0x30 | can_id, buffer, send_index);
        }
        
        // Sleep until the next scheduled time
        do {
            next_time += MS2ST(can_period_ms);
        } while (next_time <= chVTGetSystemTimeX());
        
        chThdSleepUntil(next_time);
        i++;
    }
}


// Callback function for the terminal command with arguments.
static void terminal_gene(int argc, const char **argv) {
    if (argc == 1) {
        commands_printf("Gene parameters:");
        for (size_t i = 0; i < sizeof(PARAMETERS)/sizeof(param_t); i++) {
            commands_printf("  %s: %.3f", PARAMETERS[i].name, (double)*(PARAMETERS[i].value));
        }
        commands_printf("gene_thread_loop_n: %d", gene_thread_loop_n);
        commands_printf("is_running: %d %d", thread_main_is_running, thread_can_is_running);
        commands_printf("stop_now: %d", stop_now);
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

static void terminal_motor(int argc, const char **argv) {
    commands_printf("mode %d", mode);
    commands_printf("state_can.brake_level %d", state_can.brake_level);
    commands_printf("motor_brake_current_goal %.3f", motor_brake_current_goal);
    commands_printf("motor_braking_current %.3f", motor_braking_current);
    commands_printf("motor_driving_current_goal %.3f", motor_driving_current_goal);
    commands_printf("motor_driving_current %.3f", motor_driving_current);
    commands_printf("motor_driving_current_goal_age %d", motor_driving_current_goal_age);
    commands_printf("motor_walk_direction %.3f", motor_walk_direction);

}
