/*
 * odrive_can.h - ODrive-compatible CAN protocol layer (GIM6010-8)
 *
 * Protocol (per SteadyWin GIM6010-8 manual p43-44, ODrive-compatible):
 *   CAN ID (11-bit) = (node_id << 5) | cmd_id
 *   data: 8 bytes, little-endian; floats are IEEE 754.
 *
 * Commands used:
 *   0x07  Set_Axis_State        (host→motor)  uint32 requested_state
 *   0x0B  Set_Controller_Mode   (host→motor)  uint32 control_mode, uint32 input_mode
 *   0x0E  Set_Input_Torque      (host→motor)  float  input_torque (Nm, motor side)
 *   0x09  Get_Encoder_Estimates (motor→host)  float pos_estimate, float vel_estimate
 *
 * Axis states: 1=IDLE, 3=ENCODER_OFFSET_CALIB, 4=MOTOR_CALIB, 8=CLOSED_LOOP
 * Control modes: 1=TORQUE, 2=VELOCITY, 3=POSITION
 * Input modes:   1=DIRECT (passthrough)
 */
#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

#include <stdint.h>
#include <stdbool.h>

#ifndef ODRIVE_NODE_ID
#define ODRIVE_NODE_ID  0
#endif

/* Call once at startup: initializes the MCP2515 and sends the first
 * CLOSED_LOOP + TORQUE mode request (non-blocking). Returns false only if
 * the MCP2515 itself failed to initialize. */
bool odrive_init(uint8_t node_id);

/* Re-send the CLOSED_LOOP + TORQUE mode setup. Call periodically (e.g. 1 Hz)
 * while odrive_is_closed_loop() is false, so a motor powered up after the
 * Pico still gets armed. */
void odrive_request_closed_loop(uint8_t node_id);

/* Send a motor-side torque in Nm. Called from ffb_output_torque() at ~1 kHz. */
void odrive_set_torque(uint8_t node_id, float nm);

/* Poll incoming CAN frames. Call each loop; updates internal pos/vel cache
 * from Get_Encoder_Estimates broadcasts. */
void odrive_poll(void);

/* Latest cached values (motor side, turns and turns/s). */
float odrive_get_position(void);
float odrive_get_velocity(void);

/* True if last heartbeat indicated CLOSED_LOOP. */
bool odrive_is_closed_loop(void);

/* Axis error from the last heartbeat (0 = no error). While nonzero the
 * caller should stop commanding torque. */
uint32_t odrive_axis_error(void);

#endif /* ODRIVE_CAN_H */
