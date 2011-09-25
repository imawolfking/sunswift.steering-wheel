#include <scandal/wavesculptor.h>
#include <scandal/tritium.h>
#include <scandal/message.h>
#include <project/target_config.h>

sc_time_t ws_last_drive_command_time;
extern float current_velocity;

void ws_bus_handler(float bus_current, float bus_voltage, uint32_t time) {

}

void ws_temp_handler(float hs_temp, float motor_temp, uint32_t time) {
	
}

void ws_status_handler(uint8_t rcv_err_count, uint8_t tx_err_count, uint16_t active_motor,
						uint16_t err_flags, uint16_t limit_flags, uint32_t time) {

}

void ws_velocity_handler(float vehicle_velocity, float motor_rpm, uint32_t time) {
	current_velocity = vehicle_velocity;
}

void init_ws_in_channels(void) {
	scandal_register_ws_bus_callback(&ws_bus_handler);
	scandal_register_ws_temp_callback(&ws_temp_handler);
	scandal_register_ws_status_callback(&ws_status_handler);
	scandal_register_ws_velocity_callback(&ws_velocity_handler);
}

void handle_ws_drive_commands(float velocity, float bus_current, float motor_current) {

	if (sc_get_timer() - ws_last_drive_command_time >= WS_TIME_BETWEEN_DRIVE_COMMANDS_MS) {

		scandal_send_ws_drive_command(DC_DRIVE, velocity, motor_current);
		scandal_send_ws_drive_command(DC_POWER, 0.0, bus_current);
		scandal_send_ws_id(DC_BASE, "TRIb", 4);

		ws_last_drive_command_time = sc_get_timer();

	}

}
