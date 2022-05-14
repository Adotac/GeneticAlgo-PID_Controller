import numpy as np
import math
import PID_Controller as pc

# Parameters
k = 0.2  # look forward gain
Lfc = 1.5  # [m] look-ahead distance
dt = 0.1  # [s] time tick
WB = 2  # [m] wheel base of vehicle


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return float(math.hypot(dx, dy))


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)

            ind = np.argmin(d)
            self.old_nearest_point_index = ind

        else:
            ind = self.old_nearest_point_index

            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                if (ind + 1) >= pc.LIMIT_RANGE:
                    break  # not exceed goal
                try:
                    distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                              self.cy[ind + 1])

                    if distance_this_index < distance_next_index:
                        break

                    ind = ind + 1 if (ind + 1) < pc.LIMIT_RANGE else ind
                    distance_this_index = distance_next_index

                except Exception as e:
                    print(f'{e} + \n index = {ind}')
                    exit(1)

            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            # print(f'{ind} = lookahead!!')
            if (ind + 1) >= pc.LIMIT_RANGE:
                break  # not exceed goal
            ind += 1

        return ind, Lf


def correction_control(target_c, target_ind, state, pid, last_error):
    target = state.calc_distance(target_c.cx[target_ind], target_c.cy[target_ind])
    current = state.calc_distance(state.x, state.y)
    error = target - current

    correction = lambda x, y, z: (
            (x * error) +
            ((y * (last_error + error))) +
            ((z * (error - last_error)))
    )

    # = (pid[0] * error) + ((pid[1] * (last_error + error))/error) + (pid[2] * ((error - last_error) / error))
    try:
        return correction(pid[0], pid[1], pid[2]), error
    except Exception as e:
        print(f'stops correction at {pid}')


def steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def plot_arrow(x, y, yaw, length=0.5, width=0.5, fc="r", ec="k"):
    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        pc.plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                     fc=fc, ec=ec, head_width=width, head_length=width)
        pc.plt.plot(x, y)


def graph():
    #eq = lambda x: math.sin(math.sqrt(x))  # easy
    eq = lambda x: math.sin(x / 5.0) * x  # medium
    # eq = lambda x: math.sin(x)+(x)        # hard

    x = []
    y = []
    for i in np.arange(0, pc.PLIMIT, 0.1):
        # print(i)
        x.append(i)
        y.append(round(eq(i), 2))

    return [x, y]


def fitness_simulation(pid):
    rear_x = []
    rear_y = []
    #  target course
    cgraph = graph()

    T = pc.LIMIT_RANGE  # max simulation time

    # initial state
    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

    lastIndex = len(cgraph[0]) - 1
    time = 0.0

    rear_x.append(state.x)
    rear_y.append(state.y)

    target_course = TargetCourse(cgraph[0], cgraph[1])
    target_ind, _ = target_course.search_target_index(state)

    ai = 0.0
    last_error = 0.0
    while T >= time and lastIndex > target_ind:
        # Calc control input
        ai, last_error = correction_control(target_course, target_ind, state, pid, last_error)
        di, target_ind = steer_control(
            state, target_course, target_ind)

        state.update(ai, di)  # Control vehicle

        time += dt
        rear_x.append(round(state.x, 2))
        rear_y.append(round(state.y, 2))

    return [rear_x, rear_y]


def simulation(pid, show=False):
    #  target course
    cgraph = graph()
    rear_x = []
    rear_y = []

    T = pc.LIMIT_RANGE  # max simulation time

    # initial state
    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

    lastIndex = len(cgraph[0]) - 1
    time = 0.0

    rear_x.append(state.x)
    rear_y.append(state.y)

    target_course = TargetCourse(cgraph[0], cgraph[1])
    target_ind, _ = target_course.search_target_index(state)

    ai = 0.0
    last_error = 0.0
    while T >= time and lastIndex > target_ind:

        # Calc control input
        ai, last_error = correction_control(target_course, target_ind, state, pid, last_error)
        di, target_ind = steer_control(
            state, target_course, target_ind)

        state.update(ai, di)  # Control vehicle

        time += dt
        rear_x.append(round(state.x, 2))
        rear_y.append(round(state.y, 2))

        if show:
            pc.plt.cla()

            plot_arrow(state.x, state.y, state.yaw)
            pc.plt.plot(cgraph[0], cgraph[1], "-r", label="path")
            pc.plt.plot(rear_x, rear_y, "-b", label="trajectory")
            pc.plt.plot(cgraph[0][target_ind], cgraph[1][target_ind], "xg", label="target")
            pc.plt.grid(True)
            pc.plt.pause(0.01)

    if show:
        pc.plt.close()
    return rear_x, rear_y
