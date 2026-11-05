from dataclasses import dataclass, field
import matplotlib.pyplot as plt
import numpy as np

@dataclass
class SimResult: # holds list of all motor states
    time: list = field(default_factory=list)
    theta: list = field(default_factory=list)
    omega: list = field(default_factory=list)
    omega_ref: list = field(default_factory=list)
    current: list = field(default_factory=list)
    current_ref: list = field(default_factory=list)
    command_voltage: list = field(default_factory=list)
    applied_voltage: list = field(default_factory=list)


def motor_load(t): # external load disturbance
    load = 0.4
    if t>3: load += 0.2
    if t>6: load = load - 0.4
    return load

def clip(x, low, hi):
    return low if x < low else hi if x > hi else x

def step(n,step_n,val):
    if n>step_n:
        return val
    return 0

def sfct(n, w_target, step_at_sec=2.0, duration=0.5, dt=0.001):
    """
    Generates a smooth S-curve reference trajectory list of length n steps.
    """
    step_index = int(step_at_sec / dt)
    acc_steps = int(duration / dt)

    s_list = np.zeros(n)
    for k in range(n):
        if k < step_index:
            s_list[k] = 0.0
        elif k >= step_index + acc_steps:
            s_list[k] = w_target
        else:
            tau = (k - step_index) / acc_steps
            scale = 10 * (tau ** 3) - 15 * (tau ** 4) + 6 * (tau ** 5)
            s_list[k] = w_target * scale

    return s_list.tolist()

def plot_result(r, t_limits=(0, 10)):
    """Plots one SimResult, sliced between t_limits."""

    time_arr = np.array(r.time)
    omega_arr = np.array(r.omega)
    omega_ref_arr = np.array(r.omega_ref)
    current_arr = np.array(r.current)
    current_ref_arr = np.array(r.current_ref)
    cmd_v_arr = np.array(r.command_voltage)
    app_v_arr = np.array(r.applied_voltage)

    # Create boolean mask for the desired time window
    mask = (time_arr >= t_limits[0]) & (time_arr <= t_limits[1])
    t_zoom = time_arr[mask]

    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    ax[0].plot(t_zoom, omega_arr[mask], label="omega")
    ax[0].plot(t_zoom, omega_ref_arr[mask], 'k--', label="reference")
    ax[0].set_ylabel("rad/s")

    ax[1].plot(t_zoom, current_arr[mask], label="current")
    ax[1].plot(t_zoom, current_ref_arr[mask], 'k--', label="i_ref")
    ax[1].set_ylabel("A")


    if len(r.command_voltage) == len(r.time): #if pid (it has command and applied)
        ax[2].step(t_zoom, app_v_arr[mask], where="post", label = "V_applied")
        ax[2].plot(t_zoom, cmd_v_arr[mask], label="V_command")
    else: #mpc only has applied voltage signal
        ax[2].step(t_zoom, app_v_arr[mask], where="post", label = "V_applied")

    ax[2].set_ylabel("V")
    ax[2].set_xlabel("Time (s)")


    for a in ax:
        a.grid(True)
        a.set_xlim(t_limits)

    ax[0].legend(loc="upper right")  # Fixed location avoids the slow 'best' search
    ax[1].legend(loc="upper right")  # Fixed location avoids the slow 'best' search
    ax[2].legend(loc="upper right")  # Fixed location avoids the slow 'best' search

    plt.tight_layout()
    plt.show()
