import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        # STUDENT CODE HERE

    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        r_des = flat_output['x']
        r_dot_des = flat_output['x_dot']
        r_ddot_des = flat_output['x_ddot']
        yaw_des = flat_output['yaw']
        yaw_dot_des = flat_output['yaw_dot']

        r_st = state['x']
        r_dot_st = state['v']
        quat_st = state['q']
        omega_st = state['w']

        Kp1 = 35
        Kp2 = Kp1
        Kp3 = 100
        Kd1 = 20
        Kd2 = Kd1
        Kd3 = 40

        Kp = np.diag([Kp1,Kp2,Kp3])
        Kd = np.diag([Kd1,Kd2,Kd3])

        error = r_st - r_des
        error_dot = r_dot_st - r_dot_des

        r_ddot_cont = r_ddot_des - Kd@error_dot - Kp@error

        cmd_thrust = (r_ddot_cont[2] + self.g) * self.mass
        phi_des = (r_ddot_cont[0]*np.sin(yaw_des) - r_ddot_cont[1]*np.cos(yaw_des))/self.g
        theta_des = (r_ddot_cont[0]*np.cos(yaw_des) + r_ddot_cont[1]*np.sin(yaw_des))/self.g
        psi_des = yaw_des

        euler_cmd = np.zeros((3))
        euler_cmd[0] = psi_des
        euler_cmd[1] = phi_des
        euler_cmd[2] = theta_des

        cmd_q = Rotation.from_euler('zxy', euler_cmd, degrees=False).as_quat()

        p_des = 0
        q_des = 0
        r_des = yaw_dot_des

        Rotation_angles = Rotation.from_quat(quat_st).as_euler('zxy', degrees=False)

        phi_st = Rotation_angles[1]
        psi_st = Rotation_angles[0]
        theta_st = Rotation_angles[2]

        Kp_phi = 8000
        Kd_phi = 200
        Kp_theta = Kp_phi
        Kd_theta = Kd_phi
        Kp_psi = 1000
        Kd_psi = 100

        phi_ddot = Kp_phi*(phi_des - phi_st) + Kd_phi*(p_des - omega_st[0])
        theta_ddot = Kp_theta * (theta_des - theta_st) + Kd_theta * (q_des - omega_st[1])
        psi_ddot = Kp_psi * (psi_des - psi_st) + Kd_psi * (r_des - omega_st[2])
        angle_ddot_matrix = np.array([phi_ddot,theta_ddot,psi_ddot])

        u2 = self.inertia@angle_ddot_matrix.reshape(-1,1)
        cmd_moment = u2
        U = np.vstack((np.array(cmd_thrust), u2))

        L = self.arm_length
        gamma = self.k_drag / self.k_thrust

        Force_mat = np.array([[1, 1, 1, 1],
                              [0, L, 0, -L],
                              [-L, 0, L, 0],
                              [gamma, -gamma, gamma, -gamma]])

        F = np.linalg.inv(Force_mat) @ U
        k_F = self.k_thrust  # coff of force

        for i in range(0, 4):
            if F[i, 0] < 0:
                F[i, 0] = 0
            cmd_motor_speeds[i] = np.sqrt(F[i, 0] / k_F)
            if cmd_motor_speeds[i] > self.rotor_speed_max:
                cmd_motor_speeds[i] = self.rotor_speed_max




        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
        return control_input
