import math
import matplotlib.pyplot as plt
import pyqtgraph as pg
from collections import deque

POSX = 0
POSY = 1
YAW = 2
SPD = 3
YAWSPD = 4


def plot_arrow(x, y, yaw, length=5, width=0.1):
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot(ltraj, state, goal, ob, path, traj):
    plt.cla()
    ax = plt.gca()
    if ltraj:
        data_x = [s[0] for s in ltraj]
        data_y = [s[1] for s in ltraj]
        plt.plot(data_x, data_y, color="red", linewidth=1)
    plt.plot(state[POSX], state[POSY], "xr")
    plt.plot(goal[0], goal[1], "xb", label='target')
    plt.plot(ob[:, 0], ob[:, 1], "ok", label='obstacle')
    ax.legend(loc='best')

    for i in range(len(ob)):
        if ob[i][2] != 0:
            circle = plt.Circle((ob[i, 0], ob[i, 1]), 3, color='red', Fill=False)
        else:
            circle = plt.Circle((ob[i, 0], ob[i, 1]), 3, color='blue', Fill=False)
        # plt.text(ob[i, 0], ob[i, 1], 'No. %s' % i, family='serif', style='italic', ha='right', wrap=True)
        ax.add_patch(circle)
    plt.scatter([s[0] for s in path], [s[1] for s in path], c='r', s=1)
    plt.scatter([s[0] for s in traj], [s[1] for s in traj], c='g', s=1)
    plot_arrow(state[POSX], state[POSY], state[YAW])
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.0001)

# without RRT path
def plot2(ltraj, state, goal, ob, traj):
    plt.cla()
    ax = plt.gca()
    if ltraj:
        data_x = [s[0] for s in ltraj]
        data_y = [s[1] for s in ltraj]
        plt.plot(data_x, data_y, color="red", linewidth=1)
    plt.plot(state[POSX], state[POSY], "xr")
    plt.plot(goal[0], goal[1], "xb", label='target')
    plt.plot(ob[:, 0], ob[:, 1], "ok", label='obstacle')
    ax.legend(loc='best')

    for i in range(len(ob)):
        if ob[i][2] != 0:
            circle = plt.Circle((ob[i, 0], ob[i, 1]), 3, color='red', Fill=False)
        else:
            circle = plt.Circle((ob[i, 0], ob[i, 1]), 3, color='blue', Fill=False)
        # plt.text(ob[i, 0], ob[i, 1], 'No. %s' % i, family='serif', style='italic', ha='right', wrap=True)
        ax.add_patch(circle)
    plt.scatter([s[0] for s in traj], [s[1] for s in traj], c='g', s=1)
    plot_arrow(state[POSX], state[POSY], state[YAW])
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.0001)

PID_Tuner = False


if __name__ =="__main__":

    try:

        app = pg.mkQApp()

        win = pg.GraphicsWindow()
        win.setWindowTitle(u'Sensor Monitor')
        win.resize(1600, 800)

        # CoorAxis = win.ViewBox()
        # CoorAxis.disableAutoRange()
        p1 = win.addPlot(title='Pos', row=0, col=0, labels={'left':'posx','bottom':'posy'})
        p1.showGrid(x=True, y=True)
        curve1 = p1.plot(pen=(255, 0, 0), name="Red curve")
        point0 = p1.plot(symbol='s')
        point1 = p1.plot(symbol='o')
        point2 = p1.plot(symbol='o')

        # point1 = p1.plot(pen=(0, 255, 0), symbol='o')

        p2 = win.addPlot(title='yaw', row=0, col=1, labels={'left': 'yaw(rad)', 'bottom': 'time(0.1s)'})
        p2.showGrid(x=True, y=True)
        curve2 = p2.plot(pen=(0, 255, 0), name="Green curve")

        p3 = win.addPlot(title='yaw speed', row=1, col=0, labels={'left': 'yaw speed(rad/s)', 'bottom': 'time(0.1s)'})
        p3.showGrid(x=True, y=True)
        curve3 = p3.plot(pen=(0, 255, 0), name="Green curve")

        p4 = win.addPlot(title='speed', row=1, col=1, labels={'left': 'speed(m/s)', 'bottom': 'time(0.1s)'})
        p4.showGrid(x=True, y=True)
        curve4 = p4.plot(pen=(0, 0, 255), name="Blue curve")

        cnt = 0
        data_rec = deque(maxlen=100)
        target_rec = deque(maxlen=1)
        ob1_rec = deque(maxlen=1)
        ob2_rec = deque(maxlen=1)
        count = deque(maxlen=100)

        def update():
            global cnt
            # 本船
            # tmp = interface001.receive('gps.posx', 'gps.posy', 'ahrs.yaw', 'gps.hspeed', 'ahrs.yaw_speed')
            # data_rec.append(tmp)
            # 被跟踪船
            tmp = target.receive('gps.posx', 'gps.posy')
            target_rec.append(tmp)
            # obstacle 1
            tmp = obstacle1.receive('gps.posx', 'gps.posy')
            ob1_rec.append(tmp)
            # obstacle 2
            tmp = obstacle2.receive('gps.posx', 'gps.posy')
            ob2_rec.append(tmp)

            count.append(cnt)
            # curve1.setData([-d[1] for d in data_rec], [-d[0] for d in data_rec])
            point0.setData([-target_rec[-1][1]], [-target_rec[-1][0]])
            point1.setData([-ob1_rec[-1][1]], [-ob1_rec[-1][0]])
            point2.setData([-ob2_rec[-1][1]], [-ob2_rec[-1][0]])
            # point1.setData([data_rec[-1][1]], [data_rec[-1][0]])
            # curve2.setData(count, [d[2] for d in data_rec])
            # curve3.setData(count, [d[4] for d in data_rec])
            # curve4.setData(count, [d[3] for d in data_rec])
            cnt += 1

        timer = pg.QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(100)

        app.exec_()

    except (KeyboardInterrupt, Exception) as e:
        print('closed')
    finally:
        pass
