from msgdev import MsgDevice, PeriodTimer
import pyqtgraph as pg
from collections import deque


class Interface_rec(object):
    def __init__(self, sub_addr, pose_port):
        self.dev = MsgDevice()
        self.dev.open()
        self.dev.sub_connect(sub_addr + ':' + pose_port)
        self.dev.sub_add_url('posx')
        self.dev.sub_add_url('posy')
        self.dev.sub_add_url('speed')
        self.dev.sub_add_url('speed_tar')
        self.dev.sub_add_url('yaw')
        self.dev.sub_add_url('yaw_tar')
        self.dev.sub_add_url('yawspd')
        self.dev.sub_add_url('yawspd_tar')

    def receive(self, *args):
        data = []
        for i in args:
            data.append(self.dev.sub_get1(i))
        return data


local_addr = 'tcp://127.0.0.1'
self_port = '55004'
auv = Interface_rec(local_addr, self_port)

if __name__ == "__main__":
    try:
        app = pg.mkQApp()

        win = pg.GraphicsWindow()
        win.setWindowTitle(u'Sensor Monitor')
        win.resize(1600, 800)

        # CoorAxis = win.ViewBox()
        # CoorAxis.disableAutoRange()
        p1 = win.addPlot(title='Pos', row=0, col=0, labels={'left': 'posx', 'bottom': 'posy'})
        p1.showGrid(x=True, y=True)
        curve1 = p1.plot(pen=(255, 0, 0), name="Red curve")
        point0 = p1.plot(symbol='s')
        point1 = p1.plot(symbol='o')
        point2 = p1.plot(symbol='o')

        # point1 = p1.plot(pen=(0, 255, 0), symbol='o')

        p2 = win.addPlot(title='yaw', row=0, col=1, labels={'left': 'yaw(rad)', 'bottom': 'time(0.1s)'})
        p2.showGrid(x=True, y=True)
        curve21 = p2.plot(pen=(0, 255, 0), name="Green curve")
        curve22 = p2.plot(pen=(0, 0, 255), name="Blue curve")

        p3 = win.addPlot(title='speed', row=1, col=0, labels={'left': 'speed(m/s)', 'bottom': 'time(0.1s)'})
        p3.showGrid(x=True, y=True)
        curve31 = p3.plot(pen=(0, 255, 0), name="Green curve")
        curve32 = p3.plot(pen=(0, 0, 255), name="Blue curve")

        p4 = win.addPlot(title='yaw speed', row=1, col=1, labels={'left': 'yaw speed(rad/s)', 'bottom': 'time(0.1s)'})
        p4.showGrid(x=True, y=True)
        curve41 = p4.plot(pen=(0, 255, 0), name="Green curve")
        curve42 = p4.plot(pen=(0, 0, 255), name="Blue curve")

        cnt = 0
        data_rec = deque(maxlen=100)
        target_rec = deque(maxlen=1)
        ob1_rec = deque(maxlen=1)
        ob2_rec = deque(maxlen=1)
        count = deque(maxlen=100)


        def real_time_plot():
            global cnt
            state = auv.receive('posx', 'posy', 'speed', 'yaw', 'yawspd', 'speed_tar', 'yaw_tar', 'yawspd_tar')
            data_rec.append(state)
            # target_rec.append(goal)
            # ob1_rec.append(moving_obstacles[0])
            # ob2_rec.append(moving_obstacles[1])
            count.append(cnt)

            # curve1.setData([-d[1] for d in data_rec], [-d[0] for d in data_rec])
            curve1.setData([d[0] for d in data_rec], [d[1] for d in data_rec])
            # point0.setData([-target_rec[-1][1]], [-target_rec[-1][0]])
            # point1.setData([-ob1_rec[-1][1]], [-ob1_rec[-1][0]])
            # point2.setData([-ob2_rec[-1][1]], [-ob2_rec[-1][0]])
            point1.setData([data_rec[-1][0]], [data_rec[-1][1]])
            # yaw
            curve21.setData(count, [d[3] for d in data_rec])  # real yaw angle
            curve22.setData(count, [d[6] for d in data_rec])  # target yaw angle
            # speed
            curve31.setData(count, [d[2] for d in data_rec])  # real speed
            curve32.setData(count, [d[5] for d in data_rec])  # target speed
            # yaw speed
            curve41.setData(count, [d[4] for d in data_rec])  # real yaw speed
            curve42.setData(count, [d[7] for d in data_rec])  # target yaw speed
            cnt += 1

        timer = pg.QtCore.QTimer()
        timer.timeout.connect(real_time_plot)
        timer.start(100)

        app.exec_()

    except (KeyboardInterrupt, Exception) as e:
        print('closed')
    finally:
        pass
