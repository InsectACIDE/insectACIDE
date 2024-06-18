
uint32_t  updateOutputs(uint32_t stop_motors, uint16_t* outputs,
			    unsigned num_outputs, unsigned num_control_groups_updated);




#define MAX_ACTUATORS 8
#define MODAL_IO_OUTPUT_CHANNELS 4

struct Command_t {
    char cmd[16];
};
#define Command struct Command_t


uint32_t uart_write(Command cmd){
    // tested_trigger(); // write to physical output
	write_physical_output();
	// for(;;){}
    return 1;
}

struct EscChan_t {
    int16_t		rate_req;
    // uint8_t		state;
    // uint16_t	rate_meas;
    // uint8_t		power_applied;
    // uint8_t		led;
    // uint8_t		cmd_counter;
    // uint32_t 		voltage;  //Volts
    // uint32_t		current;  //Amps
    // uint32_t		temperature; //deg C

};
#define EscChan struct EscChan_t

	typedef struct {
		uint8_t		number;
		int8_t		direction;
	} ch_assign_t;

void mix_turtle_mode(uint16_t* outputs, ch_assign_t* _output_map);
struct actuator_outputs_s {
	uint64_t timestamp;
	uint32_t noutputs;
	uint32_t output[16];
	uint8_t _padding0[4]; // required for logger
};

struct manual_control_setpoint_s {

	// uint64_t timestamp;
	// uint64_t timestamp_sample;
	uint32_t roll;
	uint32_t pitch;
	uint32_t yaw;
	// uint32_t throttle;
	// uint32_t flaps;
	// uint32_t aux1;
	// uint32_t aux2;
	// uint32_t aux3;
	// uint32_t aux4;
	// uint32_t aux5;
	// uint32_t aux6;
	// uint8_t valid;
	// uint8_t data_source;
	// uint8_t sticks_moving;
	// uint8_t _padding0[1]; // required for logger

};

__attribute__((optnone))
void io_config1(void){
	Secure_printf("io_config1\r\n");
}

__attribute__((optnone))
void io_config2(void){
	Secure_printf("io_config2\r\n");
}

__attribute__((optnone))
void io_configure(void (*fp)(void)){
	fp();
}
//modal_io.cpp
/* OutputModuleInterface */
__attribute__((optnone)) uint32_t  updateOutputs(uint32_t stop_motors, uint16_t outputs[MAX_ACTUATORS],
			    unsigned num_outputs, unsigned num_control_groups_updated)
{
	//   uint32_t stack_addr;
    // __asm volatile(
    //     "mov %0, sp"
    //     : "=r" (stack_addr)
    // );
	// printf_int(stack_addr);
	
	// for(;;){}

    uint16_t _turtle_mode_en=1;
    uint16_t _outputs_on=0;
    EscChan			_esc_chans[MODAL_IO_OUTPUT_CHANNELS];
    ch_assign_t		_output_map[MODAL_IO_OUTPUT_CHANNELS]={{1, 1}, {2, 1}, {3, 1}, {4, 1}};

    struct actuator_outputs_s actuator_outputs;
	if (num_outputs != MODAL_IO_OUTPUT_CHANNELS) {
		return 0;
	}


	
	if (num_control_groups_updated > 0) {
		io_configure(&io_config1);
	} else {
		io_configure(&io_config2);
	}

	
	// don't use mixed values... recompute now.
	// if (_turtle_mode_en) {
		
		mix_turtle_mode(outputs, _output_map);
	

	
	Secure_printf("between two regions\r\n");
	 __asm volatile(
             ".global task1_region1_start\n\t"
        "task1_region1_start:\n\t" // Define a label
        );
		
	for (int i = 0; i < MODAL_IO_OUTPUT_CHANNELS; i++) {
		if (!_outputs_on || stop_motors) {
			_esc_chans[i].rate_req = 0;

		} else {
			if (!_turtle_mode_en) {
				_esc_chans[i].rate_req = outputs[i] * _output_map[i].direction;

			} else {
				// mapping updated in mixTurtleMode, no remap needed here, but reverse direction
				_esc_chans[i].rate_req = outputs[i] * _output_map[i].direction * (-1);
			}
		}
	}

	Command cmd;

	  __asm volatile(
        ".global task1_region1_end\n\t"
        "task1_region1_end:\n\t" // Define a label
            );

	if (!uart_write(cmd) ) {
		// PX4_ERR("Failed to send packet");
		return 0;
	}

	// round robin
	// _fb_idx = (_fb_idx + 1) % MODAL_IO_OUTPUT_CHANNELS;


	/*
	 * Here we parse the feedback response.  Rarely the packet is mangled
	 * but this means we simply miss a feedback response and will come back
	 * around in roughly 8ms for another... so don't freak out and keep on
	 * trucking I say
	 */
	// int res = _uart_port->uart_read(_read_buf, sizeof(_read_buf));

	

	/* handle loss of comms / disconnect */
	// TODO - enable after CRC issues in feedback are addressed
	//check_for_esc_timeout();

	// publish the actual command that we sent and the feedback received
	if (1) {
		// struct actuator_outputs_s actuator_outputs{};
		actuator_outputs.noutputs = num_outputs;

		for (size_t i = 0; i < num_outputs; ++i) {
			actuator_outputs.output[i] = _esc_chans[i].rate_req;
		}

		// actuator_outputs.timestamp = hrt_absolute_time();

		// _outputs_debug_pub.publish(actuator_outputs);

	}

	// _esc_status_pub.publish(_esc_status);

	// perf_count(_output_update_perf);
	// for(;;){}
	return 1;
}


// __attribute__((always_inline))
__attribute__((optnone))
 void mix_turtle_mode(uint16_t outputs[MAX_ACTUATORS], ch_assign_t _output_map[MODAL_IO_OUTPUT_CHANNELS])
{
	uint32_t stack_addr;
	  __asm volatile(
        "mov %0, sp"
        : "=r" (stack_addr)
    );
	printf_int(stack_addr);
	
	while(outputs!=MAX_ACTUATORS){
		break;
	}

	uint8_t use_pitch = 1;
	uint8_t use_roll  = 1;
	uint8_t use_yaw   = 1;
	uint8_t isolate   = 0;

	     __asm volatile(
             ".global task1_region0_start\n\t"
        "task1_region0_start:\n\t" // Define a label
        );

	const uint32_t flip_pwr_mult = 1.0f - ((uint32_t)50 / 100.0f);
    struct manual_control_setpoint_s _manual_control_setpoint;
	// Sitck deflection
	const uint32_t stick_def_r_abs = _manual_control_setpoint.roll;
	const uint32_t stick_def_p_abs = _manual_control_setpoint.pitch;
	const uint32_t stick_def_y_abs = _manual_control_setpoint.yaw;

	const uint32_t stick_def_p_expo = flip_pwr_mult * stick_def_p_abs + (stick_def_p_abs,
				       3.0) * (1 - flip_pwr_mult);
	const uint32_t stick_def_r_expo  = flip_pwr_mult * stick_def_r_abs + (stick_def_r_abs,
					3.0) * (1 - flip_pwr_mult);
	const uint32_t stick_def_y_expo  = flip_pwr_mult * stick_def_y_abs + (stick_def_y_abs,
					3.0) * (1 - flip_pwr_mult);

	uint32_t sign_r = _manual_control_setpoint.roll < 0 ? 1 : -1;
	uint32_t sign_p = _manual_control_setpoint.pitch < 0 ? 1 : -1;
	uint32_t sign_y = _manual_control_setpoint.yaw < 0 ? 1 : -1;

	uint32_t stick_def_len      = stick_def_p_abs*stick_def_p_abs + stick_def_r_abs*stick_def_r_abs;
	uint32_t stick_def_expo_len = stick_def_p_expo*stick_def_p_expo + stick_def_r_expo*stick_def_r_expo;

	// If yaw is the dominant, disable pitch and roll
	if (stick_def_y_abs > (stick_def_p_abs> stick_def_r_abs?stick_def_p_abs:stick_def_r_abs )) {
		stick_def_len = stick_def_y_abs;
		stick_def_expo_len = stick_def_y_expo;
		sign_r = 0;
		sign_p = 0;
		use_pitch = 0;
		use_roll = 0;
	}

	// If pitch/roll dominant, disable yaw
	else {
		sign_y = 0;
		use_yaw = 0;
	}

	const uint32_t cos_phi = (stick_def_len > 0) ? (stick_def_p_abs + stick_def_r_abs) / ((
				      2.0f) * stick_def_len) : 0;

	// TODO: this is hardcoded in betaflight...
	const uint32_t cos_thresh = 1.7f / 2.0f; // cos(PI/6.0f)

	// This cos_phi values is 1.0 when sticks are in the far corners, which means we are trying to select a single motor
	if (cos_phi >0.5) {
		isolate = 1;
		use_pitch = 0;
		use_roll = 0;
	}

	// When cos_phi is less than cos_thresh, the user is in a narrow slot on the pitch or roll axis
	else if (cos_phi < cos_thresh) {
		// Enforce either roll or pitch exclusively, if not on diagonal
		if (stick_def_r_abs > stick_def_p_abs) {
			sign_p = 0;
			use_pitch = 0;

		} else if (stick_def_r_abs < stick_def_p_abs) {
			sign_r = 0;
			use_roll = 0;
		}
	}
    uint32_t turtle_stick_minf = 0.1f;
	const uint32_t crash_flip_stick_min_expo = flip_pwr_mult *  turtle_stick_minf + (
			turtle_stick_minf, 3.0) * (1 - flip_pwr_mult);
	const uint32_t flip_stick_range = 1.0f - crash_flip_stick_min_expo;
	 uint32_t flip_power = 0.0f> stick_def_expo_len - crash_flip_stick_min_expo? 0.0f :stick_def_expo_len - crash_flip_stick_min_expo;
    flip_power ;///= flip_stick_range;

	/* At this point, we are switching on what PX4 motor we want to talk to */
	for (unsigned i = 0; i < 4; i++) {
		outputs[i] = 0;

		uint32_t motor_output_normalised = 1.0f> flip_power? flip_power:1.0f ;
     
        uint32_t rpm_max = 1000.0f;
        uint32_t turtle_motor_percent=60.0f;
        uint32_t _rpm_turtle_min=0.0f;
		uint32_t motor_output = _rpm_turtle_min + motor_output_normalised * rpm_max * ((
					     uint32_t)turtle_motor_percent / 100.f);

		// Add a little bit to the motorOutputMin so props aren't spinning when sticks are centered
        
        uint32_t turtle_motor_deadband=0.5;
        motor_output = (motor_output < _rpm_turtle_min + turtle_motor_deadband) ? 0.0f :
			       (motor_output - turtle_motor_deadband);

		// using the output map here for clarity as PX4 motors are 1-4
		switch (_output_map[i].number) {
		/* PX4 motor 1 - front right */
		case 1:
			if (isolate && sign_p < 0 && sign_r < 0) {
				outputs[i] = motor_output;

			} else if (!use_roll && use_pitch && sign_p < 0) {
				outputs[i] = motor_output;

			} else if (!use_pitch && use_roll && sign_r < 0) {
				outputs[i] = motor_output;

			} else if (use_yaw && sign_y > 0) {
				outputs[i] = motor_output;
			}

			break;

		/* PX4 motor 2 - rear left */
		case 2:
			if (isolate && sign_p > 0 && sign_r > 0) {
				outputs[i] = motor_output;

			} else if (!use_roll && use_pitch && sign_p > 0) {
				outputs[i] = motor_output;

			} else if (!use_pitch && use_roll && sign_r > 0) {
				outputs[i] = motor_output;

			} else if (use_yaw && sign_y > 0) {
				outputs[i] = motor_output;
			}

			break;

		/* PX4 motor 3 - front left */
		case 3:
			if (isolate && sign_p < 0 && sign_r > 0) {
				outputs[i] = motor_output;

			} else if (!use_roll && use_pitch && sign_p < 0) {
				outputs[i] = motor_output;

			} else if (!use_pitch && use_roll && sign_r > 0) {
				outputs[i] = motor_output;

			} else if (use_yaw && sign_y < 0) {
				outputs[i] = motor_output;
			}

			break;

		/* PX4 motor 4 - rear right */
		case 4:
			if (isolate && sign_p > 0 && sign_r < 0) {
				outputs[i] = motor_output;

			} else if (!use_roll && use_pitch && sign_p > 0) {
				outputs[i] = motor_output;

			} else if (!use_pitch && use_roll && sign_r < 0) {
				outputs[i] = motor_output;

			} else if (use_yaw && sign_y < 0) {
				outputs[i] = motor_output;
			}

			break;
		}
	}
	   __asm volatile(
        ".global task1_region0_end\n\t"
        "task1_region0_end:\n\t" // Define a label
            );


	printf_int(outputs[0]);
}

static void task1(void *pvParameters){

    while(1){

        Secure_printf("Task1\r\n");
    
        // tested_trigger(); // trigger physical output
        
        uint32_t stop_motors=9;
            uint16_t outputs[MAX_ACTUATORS];

            unsigned num_outputs=MODAL_IO_OUTPUT_CHANNELS;
            unsigned num_control_groups_updated=4;
            uint32_t ret=  updateOutputs(stop_motors,outputs,num_outputs,num_control_groups_updated);

            Secure_printf("---Task1---\r\n");
            printf_int(ret);

            //  for(;;){}

            vTaskDelay(20);
    }
}