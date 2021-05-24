import numpy as np

class WaypointTraj(object):
    """

    """
    def __init__(self, points):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is an array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """

        # STUDENT CODE HERE
        self.points = points
        print(self.points)
        self.x = [self.points[0]]
        self.xdot = [np.zeros((3,))]

        self.speed = 1.5
        self.time_for_segments = [0.0]

        self.travel_dir = [np.zeros((3,))]
        self.no_of_points = len(self.points)
        if self.no_of_points > 1:
            for i in range(self.no_of_points-1):
                disp = points[i+1]-points[i]
                dist = np.linalg.norm(disp)
                disp_unit_vec = disp/dist
                self.travel_dir.append(disp_unit_vec)
                self.xdot.append(self.speed*disp_unit_vec)
                self.x.append(self.points[i+1])
                next_time = self.time_for_segments[i] + dist/self.speed
                self.time_for_segments.append(next_time)
        print(self.time_for_segments)




    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        # print(t)

        if self.no_of_points > 1:
            if t == 0:
                x = self.points[0]
            elif t > self.time_for_segments[-1]:
                x = self.points[-1]
                x_dot = np.zeros((3,))
            else:
                for i in range(self.no_of_points-1):
                    if t <= self.time_for_segments[i+1]:
                        x_dot = self.xdot[i+1]
                        x = self.points[i] + self.xdot[i+1]*(t-self.time_for_segments[i])
                        break

        else:
            x = self.points[0]


        flat_output = {'x': x, 'x_dot':x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
                        'yaw': yaw, 'yaw_dot':yaw_dot}
        return flat_output
