/*
 * Ported verbatim from Nautical-Private include/mec/pid_controller.h.
 * Standard-form PID: C(s) = Kp(1 + 1/(s*Ti) + s*Td), so the gains are
 * [kp, ti, td] (integral TIME and derivative TIME), not parallel-form
 * [kp, ki, kd] like sub_control.
 */

#ifndef _MARITIME_EC_PID_CONTROLLER_H
#define _MARITIME_EC_PID_CONTROLLER_H

struct pid_controller {
	/* PID gains (proportional, integral, derivative) */
	float kp, ti, td;

	/* previous error, used for derivative term */
	float prev_error;

	/* sum of errors, used for integral term */
	float integral;
};

void pid_set_gains(struct pid_controller *pid, float kp, float ti, float td);
void pid_reset(struct pid_controller *pid);
float pid_calculate(struct pid_controller *pid, float error, float dt);

#endif /* _MARITIME_EC_PID_CONTROLLER_H */
