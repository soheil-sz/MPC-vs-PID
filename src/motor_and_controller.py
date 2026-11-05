from helpers import clip

f_pwm = 5000 #  5khz
T_pwm = 1/f_pwm

f_mpc = f_pwm*2
T_mpc = 1/f_mpc

def carrier_pwm(v_ref,V_max,t):

    phase = (t % T_pwm) / T_pwm

    if phase <= 0.5:
        # up : goes from -12 to 12
        tri_wave = -12.0 + (phase / 0.5) * 24.0
    else:
        # down goes from 12 to -12)
        tri_wave = 12.0 - ((phase - 0.5) / 0.5) * 24.0

    #comparator
    if v_ref > tri_wave :
        return V_max
    else:
        return -V_max

class PID:

    def __init__(self, Kp=10.0, Ki=0, Kd=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.err_prev=0
        self.err_int=0
        self.err=0
        self.V_max = 12       # Max voltage PID can request

    def ResetPID(self):
        self.err_prev=0
        self.err_int=0

    def stepPID(self,err, dt):

        err_deriv=(err-self.err_prev)/dt
        self.err_int = self.err_int + err*dt

        self.err_prev=err

        command = self.Kp*err + self.Ki*self.err_int + self.Kd*err_deriv


        return clip(command, -self.V_max, self.V_max)


class MPC:

    def __init__(self, motor,Ts, horizon=4):
        self.motor = motor
        self.K = 0.05
        self.Ts = Ts
        self.voltage_candidates = [-V_max, V_max]
        self.sequences = list(itertools.product(self.voltage_candidates, repeat=horizon))

    def predict_next_current(self, V, current, omega):

        di_dt = (V - motor.R * current - motor.Ke * omega) / motor.L
        return current + di_dt * self.  Ts


    def stepMPC(self, motor, current_command, current_now,v_prev, omega):

        costs = []

        for sequence in self.sequences:
            cost = 0
            current = current_now
            V_last = v_prev

            for V in sequence :

                current = self.predict_next_current(V, current,omega)
                #tracking cost
                cost = cost + (current_command - current)**2

                # switching penalty
                delta_V = V - V_last
                cost += self.K * (delta_V/(2*V_max))**2
                V_last = V

            costs.append(cost)

        lowest_cost = costs.index(min(costs))

        return self.sequences[lowest_cost][0]


class Motor: #2nd order dc motor

    def __init__(self):

        # motor parameters
        self.R = 1.0    # ohm
        self.L = 0.01   # h
        self.J = 0.01   # kg·m²
        self.b = 0.005  # friction N·m·s/rad
        self.Kt = 0.05  # N·m/A
        self.Ke = self.Kt   # V·s/rad

        # State variables
        self.theta = 0.0      # Position (rad)
        self.omega = 0.0      # speed (rad/s)
        self.current = 0.0    # current


        # Time step

    def stepMotor(self,dt, V, tau_load=0.0): # Updates the motor state by 1 sim step with Euler.


        # eqs : L*di/dt + R*i + Ke*omega = V
        #       J*domega/dt + b*omega = Ki*i - tau_load

        # Acceleration from mechanical equation
        domega_dt = (self.Kt * self.current - self.b * self.omega - tau_load) / self.J

        # current derivative from electrical equation
        di_dt = (V - self.R * self.current - self.Ke * self.omega) / self.L

        # Update states with Euler
        self.omega += domega_dt * dt
        self.current += di_dt * dt
        self.theta += self.omega * dt


        state =[self.theta, self.omega, self.current]
        return state

    def ResetMotor(self):
        self.omega = 0.0
        self.current = 0.0
        self.theta = 0.0
        return True


