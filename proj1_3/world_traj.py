import numpy as np

from .graph_search import graph_search
from .occupancy_map import OccupancyMap

class WorldTraj(object):
    """

    """
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.1, 0.1, 0.1])
        self.margin = 0.35
        self.velocity = 1

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)
        print(self.path)
        occ_map = OccupancyMap(world, self.resolution, self.margin)

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.


        # If distance between 2 points in the same line is greater than epsil, add the points as the spline will not fit properly otherwise
        self.epsil = 2
        new_path = []

        new_path.append(self.path[0])
        new_path.append(self.path[1])

        vec_prev = self.path[1]-self.path[0]
        dir_prev = vec_prev/np.linalg.norm(vec_prev)
        prev_pt_idx = 1

        for i in range(1,self.path.shape[0]-2):
            vec_curr = self.path[i+1]-self.path[i]
            dir_curr = vec_curr/np.linalg.norm(vec_curr)

            if np.linalg.norm(dir_prev-dir_curr)<0.5:
                if np.linalg.norm(self.path[i+1] - self.path[prev_pt_idx])>self.epsil:
                    new_path.append(self.path[i+1])
                    prev_pt_idx = i+1
            else:
                new_path.append(self.path[i+1])
                dir_prev = dir_curr
                prev_pt_idx = i+1

        new_path.append(self.path[-1])

        self.points = np.asarray(new_path)
        self.jerk = 12


        ###Getting time intervals of each piecewise function.

        self.time_bet_segments = [0]
        self.total_time = [0]

        for i in range(self.points.shape[0]-1):
            dist = np.linalg.norm(self.points[i+1]-self.points[i])
            # self.time_bet_segments.append(dist/self.velocity)
            # self.total_time.append(self.total_time[i]+(dist/self.velocity))
            self.time_bet_segments.append((dist/self.jerk)**0.25)
            self.total_time.append(self.total_time[i]+((dist/self.jerk)**0.25))



        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE

        ## Getting coefficients for a minimum snap spline

        size = 8*(self.points.shape[0]-1)
        A = np.zeros((size,size))
        b = np.zeros((size,3))

        A[:4,:8] = [
                    [0, 0, 0, 0, 0, 0, 0, 1],
                    [0, 0, 0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 2, 0, 0],
                    [0, 0, 0, 0, 6, 0, 0, 0]
                    ]
        b[0] = self.points[0]
        t = self.time_bet_segments

        for i in range(1,self.points.shape[0]-1):


            # A[8*i-4:8*i+4,8*i-8:8*i+8] = [
            #                                 [t[i]**7, t[i]**6, t[i]**5, t[i]**4, t[i]**3, t[i]**2, t[i]**1, 1,    0, 0, 0, 0, 0, 0, 0, 0],
            #                                 [0      , 0      , 0      , 0      , 0      , 0      , 0      , 0,    0, 0, 0, 0, 0, 0, 0, 1],
            #                                 [7*t[i]**6,6*t[i]**5,5*t[i]**4,4*t[i]**3,3*t[i]**2,2*t[i],   1, 0,    0, 0, 0, 0, 0, 0,-1, 0],
            #                                 [42*t[i]**5, 30*t[i]**4, 20*t[i]**3, 12*t[i]**2,  6*t[i], 2, 0, 0,    0, 0, 0, 0, 0,-2, 0, 0],
            #                                 [210*t[i]**4,120*t[i]**3,60*t[i]**2,24*t[i], 6,           0, 0, 0,    0, 0, 0, 0,-6, 0, 0, 0],
            #                                 [840*t[i]**3,360*t[i]**2,120*t[i],                 24, 0, 0, 0, 0,    0, 0,0,-24, 0, 0, 0, 0],
            #                                 [2520*t[i]**2,720*t[i]**1,                  120, 0, 0, 0, 0, 0,       0, 0, -120, 0, 0, 0, 0, 0],
            #                                 [5040*t[i]**1, 720, 0, 0, 0, 0,                           0, 0,       0, -720, 0, 0, 0, 0, 0, 0]]

            A[8*i-4:8*i+4,8*i-8:8*i+8] = [
                                            [t[i]**7, t[i]**6, t[i]**5, t[i]**4, t[i]**3, t[i]**2, t[i]**1, 1,    0, 0, 0, 0, 0, 0, 0, 0],
                                            [0      , 0      , 0      , 0      , 0      , 0      , 0      , 0,    0, 0, 0, 0, 0, 0, 0, 1],
                                            [7*t[i]**6,6*t[i]**5,5*t[i]**4,4*t[i]**3,3*t[i]**2,2*t[i],   1, 0,    0, 0, 0, 0, 0, 0,-1, 0],
                                            [42*t[i]**5, 30*t[i]**4, 20*t[i]**3, 12*t[i]**2,  6*t[i], 2, 0, 0,    0, 0, 0, 0, 0,-2, 0, 0],
                                            [210*t[i]**4,120*t[i]**3,60*t[i]**2,24*t[i], 6,           0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0],
                                            [840*t[i]**3,360*t[i]**2,120*t[i],                 24, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0],
                                            [0      , 0      , 0      , 0      , 0      , 0      , 0      , 0,    0, 0, 0, 0, 6, 0, 0, 0],
                                            [0      , 0      , 0      , 0      , 0      , 0      , 0      , 0,    0, 0, 0,24, 0, 0, 0, 0]]
            b[8*i-4] = self.points[i]
            b[8*i-3] = self.points[i]

        end_time = self.time_bet_segments[-1]

        A[-4:,-8:] = [
                        [end_time**7, end_time**6, end_time**5, end_time**4, end_time**3, end_time**2, end_time**1, 1],
                        [7*end_time**6,6*end_time**5,5*end_time**4,4*end_time**3,3*end_time**2,2*end_time,   1, 0],
                        [42*end_time**5, 30*end_time**4, 20*end_time**3, 12*end_time**2,  6*end_time, 2, 0, 0],
                        [210*end_time**4,120*end_time**3,60*end_time**2,24*end_time, 6,           0, 0, 0]]
        b[-4] = self.points[-1]
        self.coefficients = np.linalg.solve(A,b)

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

        if t>self.total_time[-1]:
            x = self.points[-1]
            x_dot    = np.zeros((3,))
            flat_output = {'x': x, 'x_dot':x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
                        'yaw': yaw, 'yaw_dot':yaw_dot}
            return flat_output

        for i in range(len(self.total_time)-1):
            if t<self.total_time[i+1]:
                delta_t = t - self.total_time[i]

                x = np.array([delta_t**7, delta_t**6, delta_t**5, delta_t**4, delta_t**3, delta_t**2, delta_t**1, 1])@self.coefficients[8*i:8*i+8]
                x_dot = np.array([7*delta_t**6,6*delta_t**5,5*delta_t**4,4*delta_t**3,3*delta_t**2,2*delta_t,   1, 0])@self.coefficients[8*i:8*i+8]
                x_ddot = np.array([42*delta_t**5, 30*delta_t**4, 20*delta_t**3, 12*delta_t**2,  6*delta_t, 2, 0, 0])@self.coefficients[8*i:8*i+8]
                x_dddot = np.array([210*delta_t**4,120*delta_t**3,60*delta_t**2,24*delta_t, 6,           0, 0, 0])@self.coefficients[8*i:8*i+8]
                x_ddddot = np.array([840*delta_t**3,360*delta_t**2,120*delta_t,                 24, 0, 0, 0, 0])@self.coefficients[8*i:8*i+8]
                break

        flat_output = {'x': x, 'x_dot':x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
                        'yaw': yaw, 'yaw_dot':yaw_dot}
        return flat_output
