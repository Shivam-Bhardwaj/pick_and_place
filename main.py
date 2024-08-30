import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re

import serial
import time

# 全局变量(当前坐标)
current_actual = [-1]
algorithm_queue = -1
enableStatus_robot = -1
robotErrorState = False
globalLockValue = threading.Lock()


def ConnectRobot():
    try:
        ip = "169.254.222.125"
        dashboardPort = 29999
        movePort = 30003
        feedPort = 30004
        print("正在建立连接...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        move = DobotApiMove(ip, movePort)
        feed = DobotApi(ip, feedPort)
        print(">.<连接成功>!<")
        return dashboard, move, feed
    except Exception as e:
        print(":(连接失败:(")
        raise e


def RunPoint(move: DobotApiMove, point_list: list):
    move.MovL(point_list[0], point_list[1], point_list[2],
              point_list[3], point_list[4], point_list[5])


def GetFeed(feed: DobotApi):
    global current_actual
    global algorithm_queue
    global enableStatus_robot
    global robotErrorState
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0
        feedInfo = np.frombuffer(data, dtype=MyType)
        if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
            globalLockValue.acquire()
            # Refresh Properties
            current_actual = feedInfo["tool_vector_actual"][0]
            algorithm_queue = feedInfo['run_queued_cmd'][0]
            enableStatus_robot = feedInfo['enable_status'][0]
            robotErrorState = feedInfo['error_status'][0]
            globalLockValue.release()
        sleep(0.001)


def WaitArrive(point_list):
    while True:
        is_arrive = True
        globalLockValue.acquire()
        if current_actual is not None:
            for index in range(4):
                if (abs(current_actual[index] - point_list[index]) > 1):
                    is_arrive = False
            if is_arrive:
                globalLockValue.release()
                return
        globalLockValue.release()
        sleep(0.001)


def ClearRobotError(dashboard: DobotApiDashboard):
    global robotErrorState
    dataController, dataServo = alarmAlarmJsonFile()    # 读取控制器和伺服告警码
    while True:
        globalLockValue.acquire()
        if robotErrorState:
            numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
            numbers = [int(num) for num in numbers]
            if (numbers[0] == 0):
                if (len(numbers) > 1):
                    for i in numbers[1:]:
                        alarmState = False
                        if i == -2:
                            print("机器告警 机器碰撞 ", i)
                            alarmState = True
                        if alarmState:
                            continue
                        for item in dataController:
                            if i == item["id"]:
                                print("机器告警 Controller errorid", i,
                                      item["zh_CN"]["description"])
                                alarmState = True
                                break
                        if alarmState:
                            continue
                        for item in dataServo:
                            if i == item["id"]:
                                print("机器告警 Servo errorid", i,
                                      item["zh_CN"]["description"])
                                break

                    choose = input("输入1, 将清除错误, 机器继续运行: ")
                    if int(choose) == 1:
                        dashboard.ClearError()
                        sleep(0.01)
                        dashboard.Continue()

        else:
            if int(enableStatus_robot) == 1 and int(algorithm_queue) == 0:
                dashboard.Continue()
        globalLockValue.release()
        sleep(5)


if __name__ == '__main__':
    dashboard, move, feed = ConnectRobot()
    feed_thread = threading.Thread(target=GetFeed, args=(feed,))
    feed_thread.daemon = True
    feed_thread.start()
    feed_thread1 = threading.Thread(target=ClearRobotError, args=(dashboard,))
    feed_thread1.daemon = True
    feed_thread1.start()
    print("开始使能...")
    dashboard.EnableRobot()
    print("完成使能:)")
    print("循环执行...")
    # point_a = [0.624630,636.639160,417.777496,-179.540359,-0.041757,-1.677706]
    # point_b = [16.497187,800.273499,35.777496,179.839279,-0.243649,-1.676358]
    # point_c = [-636.632202,-3.044780,417.777496,-179.540359,-0.041757,88.652534]
    # point_d = [-636.632202,-3.044780,56.267506,-179.540359,-0.041757,88.652534]


    # arduinoPort = "COM5"
    # baudRate = 115200
    # timeout = 1
    # ser = serial.Serial(arduinoPort, baudRate, timeout=timeout)
    # time.sleep(2)

    # activateCommand = '1'
    # releaseCommand = '2'
    # resetCommand = '3'
    # ser.write(resetCommand.encode())
    while True:
        # # Point A (original point)
        # RunPoint(move, point_a)
        # WaitArrive(point_a)
        #
        # # # Point B (Workpiece point)
        # RunPoint(move, point_b)
        # WaitArrive(point_b)
        # ser.write(activateCommand.encode())  # activate valve
        # time.sleep(1)
        #
        # # Point C (destination point with Z offset)
        # RunPoint(move, point_c)
        # WaitArrive(point_c)
        #
        # # Point D (destination point)
        # RunPoint(move, point_d)
        # WaitArrive(point_d)
        # ser.write(releaseCommand.encode())
        # time.sleep(1)
        #
        # # Point C (destination point with Z offset)
        # RunPoint(move, point_c)
        # WaitArrive(point_c)
        #
        # # Point D (destination point)
        # RunPoint(move, point_d)
        # WaitArrive(point_d)
        # ser.write(activateCommand.encode())
        # time.sleep(1)
        #
        # # Point C (destination point with Z offset)
        # RunPoint(move, point_c)
        # WaitArrive(point_c)
        #
        # # Point A (original point)
        # RunPoint(move, point_a)
        # WaitArrive(point_a)
        #
        # # # Point B (Workpiece point)
        # RunPoint(move, point_b)
        # WaitArrive(point_b)
        # ser.write(releaseCommand.encode())  # activate valve
        # time.sleep(1)
        #
        # # Point A (original point)
        # RunPoint(move, point_a)-
        # WaitArrive(point_a)

        # zero posotion 
        zeroPosition = ([-39.764706,547.754089,582.631653,179.624405,0.851576,-91.122894])
        # # offset = [-0.02093124 * 1000, -0.00694516 * 1000, -0.01624545 * 1000, 0, 0, 0]
        offset2 =[-0.21851793*1000,  0.1222258*-1000, -(582-81), 0, 0, 0, 0] # [ 0.18245879 -0.01815373 -0.04445153]
        calibratedPos = [0] * 6
        suctionPosition = [51.291100,705.200500,582.631653,179.624405,0.851576,-91.122894]
        suctionOffset = [x - y for x, y in zip(zeroPosition, suctionPosition)] 
        for i in range(6): 
            calibratedPos[i] = zeroPosition[i] + offset2[i] - suctionOffset[i]
        RunPoint(move, zeroPosition)
        WaitArrive(zeroPosition)
        # suction_offset = [51.291100,705.200500,91.198929,179.624405,0.851576,-91.122894] #
        
        dashboard.GetPose()
        time.sleep(10)