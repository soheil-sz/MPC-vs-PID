import matplotlib.pyplot as plt
import numpy as np
import itertools
from dataclasses import dataclass, field

from helpers import SimResult, step, motor_load, clip, plot_result, sfct
from motor_and_controller import Motor, PID, carrier_pwm , MPC


t_sim = 10
t_plot = (0, t_sim) # plot between t_plot[0] and t_plot[1]
dt = 1e-5 #simulation freq = 10kHz
steps = int(t_sim/dt)
print(steps)

V_max = 12 # dc bus voltage

def run_simulation(motor, controller_type, omega_target):

    voltage_command = 0.0
    time = [0.0]
    theta = [0.0]
    omega = [0.0]
    omega_ref = [0.0]
    current = [0.0]
    current_ref = [0.0]
    command_voltage = [0.0]
    applied_voltage = [0.0]

    t=0

    motor.ResetMotor()
    scurve= sfct(steps, w_target=omega_target, step_at_sec=2.0, duration=1, dt=dt)

    if controller_type == "PID":

        # outer loop pole cancellation
        J = motor.J
        b = motor.b
        Kt = motor.Kt

        p1 = -b/J
        p2 = 3 *p1

        outer_Kp = (-J *(p1+p2) - b)/Kt
        outer_Ki = (J *p1*p2)/Kt

        # Initialize outer PID
        outer_PID = PID(Kp=outer_Kp, Ki=outer_Ki, Kd=0.0)

        # inner loop pole cancellation
        R = motor.R
        L = motor.L

        p1 = -R/L
        p2 = 3* p1

        inner_Kp = -L*(p1+p2)-R
        inner_Ki = L*p1*p2

        inner_PID = PID(Kp=inner_Kp, Ki=inner_Ki, Kd=0.0)


        for k in range(steps):

            #omega_cons = step(k, steps/5, omega_target)
            omega_cons = scurve[k]
            #control loop

            omega_now = motor.omega
            err_w =omega_cons-omega_now

            i_command = outer_PID.stepPID(err_w,dt)
            _command = clip(i_command, -V_max/motor.R , V_max/motor.R)
            #i_command = step(k,steps/5,3)

            i_now = motor.current
            err_i = i_command - i_now
            voltage_command = inner_PID.stepPID(err_i, dt)

            voltage_command = clip(voltage_command, -V_max, V_max)

            V_applied = carrier_pwm(voltage_command,V_max, k*dt)
            # Motor Step
            motor_state = motor.stepMotor(dt, V_applied, 0)

            #log
            time.append(t)
            theta.append(motor_state[0])

            omega.append(omega_now)
            omega_ref.append(omega_cons)

            current.append(i_now)
            current_ref.append(i_command)

            command_voltage.append(voltage_command)
            applied_voltage.append(V_applied)

            t=t+dt

    if controller_type == "MPC":
        # outer loop pole cancellation

        J = motor.J
        b = motor.b
        Kt = motor.Kt

        # Desired poles
        p1 = -b/J
        p2 = 3 *p1

        outer_Kp = (-J *(p1+p2) - b)/Kt
        outer_Ki = (J *p1*p2)/Kt

        # Initialize outer PID
        outer_PID = PID(Kp=outer_Kp, Ki=outer_Ki, Kd=0.0)

        mpc = MPC(motor,4)
        f_mpc = f_pwm*2
        T_mpc = 1/f_mpc
        mpc.Ts = T_mpc


        last_mpc_t = 0 #last time mpc switched
        V_prev = 0 #for mpc

        for k in range(steps):

           #omega_cons = step(k, steps/5, omega_target)
            omega_cons = scurve[k]
            #control loop

            omega_now = motor.omega
            err_w =omega_cons-omega_now

            #i_command = step(k,2*steps/5,3)

            i_command = outer_PID.stepPID(err_w,dt)
            i_command = clip(i_command, -V_max/motor.R , V_max/motor.R)

            i_now = motor.current

            if t-last_mpc_t > T_mpc:
                V_applied = mpc.stepMPC(motor, i_command, i_now, V_prev ,omega_now,t)
                V_prev = V_applied
                last_mpc_t = t
            else:
                V_applied = V_prev

            # Motor Step
            motor_state = motor.stepMotor(dt, V_applied, 0)

            #log
            time.append(t)
            theta.append(motor_state[0])

            omega.append(omega_now)
            omega_ref.append(omega_cons)

            current.append(i_now)
            current_ref.append(i_command)

            applied_voltage.append(V_applied)

            t=t+dt

    return SimResult(time, theta, omega, omega_ref, current, current_ref, command_voltage, applied_voltage )

motor = Motor()
r = run_simulation(motor, "PID", 30)
plot_result(r, t_plot)
