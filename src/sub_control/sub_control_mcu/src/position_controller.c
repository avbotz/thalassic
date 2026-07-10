/*
 * position_controller.c
 *
 * Ported verbatim from Nautical-Private src/position_controller.c.
 *
 * Implements a P controller to correct error in vehicle position.
 * NED position error is rotated into the body frame and fed through
 * per-axis PIDs to produce a body velocity setpoint, which the velocity
 * controller turns into force setpoints for the mixer.
 *
 * @author Kalyan Sriram, Vincent Wang
 */

#include <mec/control.h>
#include <mec/util.h>
#include <mec/pid_controller.h>

/*
 * the position controller operates as a proportional only controller
 * (since the velocity controller uses PID) so just keep the
 * integral and derivative constants to 0
 */

void position_controller_init(struct position_controller *ctrl)
{
	pid_set_gains(&ctrl->pid[0], 0.6, 0.0, 0.0);
	pid_set_gains(&ctrl->pid[1], 0.6, 0.0, 0.0);
	pid_set_gains(&ctrl->pid[2], 0.55, 0.0, 0.0);

	ctrl->use_floor_altitude = false;
}

void position_controller_update_sp(struct position_controller *ctrl, struct mec_vehicle_position *pos_sp)
{
	ctrl->position_sp.north = pos_sp->north;
	ctrl->position_sp.east = pos_sp->east;
	ctrl->position_sp.down = pos_sp->down;
	ctrl->position_sp.altitude = pos_sp->altitude;
}

void position_controller_update(struct position_controller *ctrl, struct mec_vehicle_position *pos,
		struct mec_vehicle_attitude *att, struct mec_vehicle_velocity_body *output, float dt)
{
	struct mec_vehicle_position ned_error;

	ned_error.north = ctrl->position_sp.north - pos->north;
	ned_error.east = ctrl->position_sp.east - pos->east;
	if (ctrl->use_floor_altitude)
		ned_error.down = pos->altitude - ctrl->position_sp.altitude;
	else
		ned_error.down = ctrl->position_sp.down - pos->down;

	struct mec_vehicle_position_body error;
	position_ned_to_body(
		&error,
		&ned_error,
		att);

	float max_speed = 1.;

	output->forward_m_s = normalize(
		pid_calculate(
			&ctrl->pid[0],
			error.forward,
			dt),
		-max_speed,
		max_speed);

	output->right_m_s = normalize(
		pid_calculate(
			&ctrl->pid[1],
			error.right,
			dt),
		-max_speed,
		max_speed);

	output->down_m_s = normalize(
		pid_calculate(
			&ctrl->pid[2],
			error.down,
			dt),
		-max_speed,
		max_speed);
}
