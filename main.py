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

def robot_movement(_movement = None): 
    if _movement is None: 
        return
    

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


    # arduinoPort = "/dev/ttyACM0"
    # baudRate = 115200
    # timeout = 1
    # ser = serial.Serial(arduinoPort, baudRate, timeout=timeout)
    # time.sleep(2)

    # activateCommand = '2'
    # releaseCommand = '1'
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

        # zero_position = ([-20.064650,565.365784,578.577271,179.839401,0.334552,-91.637856])
        # place_zero_pos = [-563.968567,44.502773,578.577271,179.839401,0.334552,-8.182272]
        # place_top_pos = [-617.008545,44.502773,578.577271,179.839401,0.334552,-8.182272]
        # place_pos = [-617.008545,44.502773,61.118225,179.839401,0.334552,-8.182272] 
        
        # # move to zero posotion 
        # RunPoint(move, zero_position)
        # WaitArrive(zero_position)
        
        # # calculate the suction offset
        # suctionPosition = [67.508217,660.830994,78.468628,179.839401,0.334552,-91.637856]
        # suctionOffset = [x - y for x, y in zip(zero_position, suctionPosition)]      
        
        # # get the work piece position
        # work_piece_tracker = Robot_Mapping()
        # work_piece_offset = work_piece_tracker.tracking_2()
        
        # print(work_piece_offset)
        # time.sleep(1)

        # target_position = [x for x in zero_position]      
        # for i in range(6): 
        #     target_position[i] = zero_position[i] - suctionOffset[i]
        # for i in range(len(work_piece_offset)):
        #     target_position[i] = target_position[i] - work_piece_offset[i]
        
        # target_top_position = [x for x in target_position]
        # target_top_position[2] = 150
        
        # # move to the target top position
        # RunPoint(move, target_top_position)
        # WaitArrive(target_top_position)
        
        # ser.write(activateCommand.encode())  # activate valve
        
        # # move to the target position
        # RunPoint(move, target_position)
        # WaitArrive(target_position)
        
        
        # # move back to the target top position
        # RunPoint(move, target_top_position)
        # WaitArrive(target_top_position)
        
        # # move back to zero position
        # RunPoint(move, zero_position)
        # WaitArrive(zero_position)
        
        # RunPoint(move, place_zero_pos)
        # WaitArrive(place_zero_pos)
        
        # RunPoint(move, place_top_pos)
        # WaitArrive(place_top_pos)
        
        # RunPoint(move, place_pos)
        # WaitArrive(place_pos)
        
        # ser.write(releaseCommand.encode())  # activate valve
        # sleep(0.8)
        
        # RunPoint(move, place_zero_pos)
        # WaitArrive(place_zero_pos)
        
        # RunPoint(move, zero_position)
        # WaitArrive(zero_position)
        
        # zero_pos = [82.732903,515.261353,695.823425,-178.826981,-1.433095,-3.567428]
        # RunPoint(move, zero_pos)
        # WaitArrive(zero_pos)
        
        # point_A = [32.732903,515.261353,695.823425,-178.826981,-1.433095,-3.567428]
        # point_B = [132.732903,515.261353,695.823425,-178.826981,-1.433095,-3.567428]
        # while True: 
        #     RunPoint(move, point_A)
        #     WaitArrive(point_A)
            
        #     RunPoint(move, point_B)
        #     WaitArrive(point_B)
        
        # zero position
        robot_zero_position = [84.346947,463.758148,643.298584,179.513504,0.671409,178.814194]
        workspace_zero_robot_position = [311.455750,417.828552,643.298584,179.513504,0.671409,178.814194]
        workpiece_position = [-609.15, 133.68, 80]
        
        workpiece_robot_position_offset = [x for x in workspace_zero_robot_position]
        for i in range(2): 
            workpiece_robot_position_offset[i] += workpiece_position[i]
        workpiece_robot_position_offset[2] = 200
        
        workpiece_robot_position = [x for x in workspace_zero_robot_position]
        for i in range(2): 
            workpiece_robot_position[i] += workpiece_position[i]
        workpiece_robot_position[2] = workpiece_position[2]
        
        
        
        RunPoint(move, robot_zero_position)
        WaitArrive(robot_zero_position)
        
        time.sleep(1)
        
        # RunPoint(move, workpiece_robot_position_offset)
        # WaitArrive(workpiece_robot_position_offset)
        
        # time.sleep(1)
        
        # RunPoint(move, workpiece_robot_position)
        # WaitArrive(workpiece_robot_position)
        
        # time.sleep(1)
        
            
        dashboard.GetPose()
        time.sleep(2)