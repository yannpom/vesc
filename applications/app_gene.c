/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "general.h"

// Some useful includes
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

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


// 10000 eRPM = cadence 60

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void terminal_debug(int argc, const char **argv);
static void terminal_pid(int argc, const char **argv);
static void terminal_current(int argc, const char **argv);
static void terminal_rpm(int argc, const char **argv);
static void terminal_goal(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Live data
static float actual_rpm = 0.0f;
// static volatile float actual_rpm_fast = 0.0f;
static float goal_rpm = 6000.0f;
static float e_rpm = 0.0f;
static float e_rpm_prev = 0.0f;
// static volatile float e_rpm_fast = 0.0f;
static float cmd_current = 0.0f;
static bool power_on = false;
// static volatile int power_off_tick = 0;

static float p_term = 0.0f;
static float i_term = 0.0f;
static float d_term = 0.0f;
static float d_term_filtered = 0.0f;
static float d_term_filtered2 = 0.0f;
static Biquad d_term_filter = {};


// Config
static volatile float rpm_on = 500.0f;
static volatile float rpm_off = 250.0f;
static volatile float min_brake_current = 0.1f;
static volatile float max_brake_current = 10.0f;
static volatile float pid_p = 0.005f; // Current / RPM
static volatile float pid_i = 0.2f;
static volatile float pid_d = 0.0002f;
static volatile float d_lpf_freq = 80.0f;
static volatile float d_filter_coeff = 0.2f;



#define TRUNC_FLOAT_RANGE(val, min, max) (val < min ? min : (val > max ? max : val))

// 500 Hz
#define LOOP_PERIOD 0.002f
#define LOOP_PERIOD_MS ((int)(1000*LOOP_PERIOD))



// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	// mc_interface_set_pwm_callback(pwm_callback);

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"gene_debug",
			"Print the Gene debug stuff",
			0,
			terminal_debug);
	terminal_register_command_callback(
			"gene_pid",
			"Get/Set Gene PID Gains",
			0,
			terminal_pid);
	terminal_register_command_callback(
			"gene_current",
			"Get/Set Gene Currents",
			0,
			terminal_current);
	terminal_register_command_callback(
			"gene_rpm",
			"Get/Set Gene RPM ON/OFF",
			0,
			terminal_rpm);
	terminal_register_command_callback(
		"gene_goal",
		"Get/Set Gene GOAL RPM",
		0,
		terminal_goal);

	biquad_config(&d_term_filter, BQ_LOWPASS, LOOP_PERIOD*d_lpf_freq);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	mc_interface_set_pwm_callback(0);
	terminal_unregister_callback(terminal_debug);

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
	commands_plot_add_graph("RPM");
	commands_plot_add_graph("Current");
	commands_plot_add_graph("P Term");
	commands_plot_add_graph("I Term");
	commands_plot_add_graph("D Term");
	commands_plot_add_graph("D Term filtered");

	systime_t next_time = chVTGetSystemTimeX();  // Get current system time

	int loop_n = 0;
	while (true) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}
		timeout_reset(); // Reset timeout if everything is OK.

		// RPM error
		actual_rpm = mc_interface_get_rpm();
		// actual_rpm_fast = mcpwm_foc_get_rpm_fast();

		e_rpm = actual_rpm - goal_rpm;
		// e_rpm_fast = actual_rpm_fast - goal_rpm;
		
		p_term = pid_p * e_rpm;

		d_term = pid_d * (e_rpm - e_rpm_prev) / LOOP_PERIOD;

		d_term_filtered2 = biquad_process(&d_term_filter, d_term);
		d_term_filtered += d_filter_coeff * (d_term - d_term_filtered);

		i_term += LOOP_PERIOD*(pid_i * e_rpm);
		i_term = TRUNC_FLOAT_RANGE(i_term, 0, max_brake_current);

		e_rpm_prev = e_rpm;

		cmd_current = p_term + d_term_filtered + i_term;
		cmd_current = TRUNC_FLOAT_RANGE(cmd_current, min_brake_current, max_brake_current);

		if (actual_rpm > rpm_on) {
			power_on = true;
			// power_off_tick = 0;
		} else if (power_on && actual_rpm < rpm_off) {
			// power_off_tick++;
			mc_interface_release_motor();
			power_on = false;
		}

		// if (power_on && power_off_tick > 100) {
		// 	mc_interface_release_motor();
		// 	power_on = false;
		// }

		if (power_on) {
			mc_interface_set_brake_current(cmd_current);
		}
		
		if (power_on) {
			commands_plot_set_graph(0);
			commands_send_plot_points(loop_n, 0.001f*actual_rpm);
			commands_plot_set_graph(1);
			commands_send_plot_points(loop_n, cmd_current);
			commands_plot_set_graph(2);
			commands_send_plot_points(loop_n, p_term);
			commands_plot_set_graph(3);
			commands_send_plot_points(loop_n, i_term);
			commands_plot_set_graph(4);
			commands_send_plot_points(loop_n, d_term);
			commands_plot_set_graph(5);
			commands_send_plot_points(loop_n, d_term_filtered);
		}

		next_time += MS2ST(LOOP_PERIOD_MS);
    	// Sleep until the next scheduled time
    	chThdSleepUntil(next_time);
		loop_n++;
	}
}

// Callback function for the terminal command with arguments.
static void terminal_debug(int argc, const char **argv) {
	commands_printf("rpm: %.1f", (double)actual_rpm);
	commands_printf("cmd_current: %.1f", (double)cmd_current);
}

static void terminal_pid(int argc, const char **argv) {	
	if (argc == 3) {
		const char* param = argv[1];
		float value = atof(argv[2]);
		if (strcasecmp(param, "P") == 0) {
			pid_p = value;
		} else if (strcasecmp(param, "I") == 0) {
			pid_i = value;
		} else if (strcasecmp(param, "D") == 0) {
			pid_d = value;
		} else if (strcasecmp(param, "C") == 0) {
			d_filter_coeff = value;
		} else if (strcasecmp(param, "F") == 0) {
			d_lpf_freq = value;
			biquad_config(&d_term_filter, BQ_LOWPASS, LOOP_PERIOD*d_lpf_freq);
		} else {
			commands_printf("Invalid parameter");
			return;
		}
	} else if (argc != 1) {
		commands_printf("Usage: gene_pid [P|I|D|F|C] [value]");
		return;
	}
	commands_printf("PID P=%f I=%f D=%f D_LPF_FREQ F=%f C=%f", (double)pid_p, (double)pid_i, (double)pid_d, (double)d_lpf_freq, (double)d_filter_coeff);
}

static void terminal_current(int argc, const char **argv) {
	if (argc == 3) {
		min_brake_current = atof(argv[1]);
		max_brake_current = atof(argv[2]);
	} else if (argc != 1) {
		commands_printf("Usage: gene_current [min] [max]");
		return;
	}
	commands_printf("Current min=%.1f max=%.1f", (double)min_brake_current, (double)max_brake_current);
}

static void terminal_rpm(int argc, const char **argv) {	
	if (argc == 3) {
		rpm_on = atof(argv[1]);
		rpm_off = atof(argv[2]);
	} else if (argc != 1) {
		commands_printf("Usage: gene_rpm [value_on] [value_off]");
		return;
	}
	commands_printf("RPM on=%.1f off=%.1f", (double)rpm_on, (double)rpm_off);
}


static void terminal_goal(int argc, const char **argv) {	
	if (argc == 2) {
		goal_rpm = atof(argv[1]);
	} else if (argc != 1) {
		commands_printf("Usage: gene_goal [goal_rpm]");
		return;
	}
	commands_printf("GOAL RPM %.1f", (double)goal_rpm);
}

