import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from time import sleep
import numpy as np
import matplotlib.pyplot as plt


# 全局变量(当前坐标)
current_actual = None


def connect_robot():
    try:
        ip = "192.168.5.1"
        dashboard_p = 29999
        move_p = 30003
        feed_p = 30004
        print("正在建立连接...")
        dashboard = DobotApiDashboard(ip, dashboard_p)
        move = DobotApiMove(ip, move_p)
        feed = DobotApi(ip, feed_p)
        print(">.<连接成功>!<")
        return dashboard, move, feed
    except Exception as e:
        print(":(连接失败:(")
        raise e


def run_point(move: DobotApiMove, point_list: list):
    move.MovL(point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5])


def run_arc(move: DobotApiMove, point_list: list, point_list1: list):
    move.Arc(point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5], point_list1[0],
             point_list1[1], point_list1[2], point_list1[3], point_list1[4], point_list1[5])


def get_feed(feed: DobotApi):
    global current_actual
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0

        a = np.frombuffer(data, dtype=MyType)
        if hex((a['test_value'][0])) == '0x123456789abcdef':
            # Refresh Properties
            current_actual = a["tool_vector_actual"][0]
            print("tool_vector_actual:", current_actual)

        sleep(0.001)


def wait_arrive(point_list):
    global current_actual
    while True:
        is_arrive = True

        if current_actual is not None:
            for index in range(len(current_actual)):
                if (abs(current_actual[index] - point_list[index]) > 1):
                    is_arrive = False

            if is_arrive:
                return

        sleep(0.001)


def point_interpolation(theta0, thetaf, t):
    cnt = len(t)
    tf = cnt
    print(cnt)
    dis = []
    speed = []
    acceler = []
    for i in range(cnt):
        t1 = i + 1
        dthetaf = 0
        dtheta0 = 0
        a0 = theta0
        a1 = dtheta0
        a2 = (3 / (tf ** 2)) * (thetaf - theta0) - (2 / tf) * dtheta0 - (1 / tf) * dthetaf
        a3 = (-2 / (tf ** 3)) * (thetaf - theta0) + (1 / (tf ** 2)) * (dtheta0 + dthetaf)

        dis.append(a0 + a1 * t1 + a2 * (t1 ** 2) + a3 * (t1 ** 3))
        speed.append(a1 + 2 * a2 * t1 + 3 * a3 * (t1 ** 2))
        acceler.append(2 * a2 + 6 * a3 * t1)

    return dis, speed, acceler


if __name__ == '__main__':
    dashboard, move, feed = connect_robot()
    print("开始上电...")
    dashboard.PowerOn()
    print("请耐心等待,机器人正在努力启动中...")
    count = 3
    while count > 0:
        print(count)
        count = count - 1
        sleep(1)
    print("开始使能...")
    dashboard.EnableRobot()
    print("完成使能:)")
    feed_thread = threading.Thread(target=get_feed, args=(feed,))
    feed_thread.setDaemon(True)
    feed_thread.start()

    dashboard.SpeedFactor(40)
    dashboard.SpeedL(40)
    dashboard.CP(80)
    dashboard.SpeedJ(40)

    print("循环执行...")
    move.ServoJ(0, -30, -100, 50, 90, -90)
    sleep(10)
    dt = 0.1
    t = np.arange(0, 3, dt)
    print(t)
    theta_origin = 0
    theta_final = 45
    dis, speed, acc = point_interpolation(theta_origin, theta_final, t)
    print(dis)
    print(speed)
    print(acc)


    for i in range(6):
        move.ServoJ(dis[i], -30,-100,50,90,-90)
        # wait_arrive(point_b)

    plt.figure()

    # 绘制位移曲线
    plt.subplot(3, 1, 1)
    plt.plot(t, dis)
    plt.xlabel('Time')
    plt.ylabel('Displacement')
    plt.title('Displacement over Time')

    # 绘制速度曲线
    plt.subplot(3, 1, 2)
    plt.plot(t, speed)
    plt.xlabel('Time')
    plt.ylabel('Speed')
    plt.title('Speed over Time')

    # 绘制加速度曲线
    plt.subplot(3, 1, 3)
    plt.plot(t, acc)
    plt.xlabel('Time')
    plt.ylabel('Acceleration')
    plt.title('Acceleration over Time')

    # 调整子图之间的间距
    plt.tight_layout()

    # 显示图形
    plt.show()

