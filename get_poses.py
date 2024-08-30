import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re
import cv2
import os
from pypylon import pylon

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

def robot_pose_parser(input_string):
    pattern = r"\{([^}]+)\}"
    match = re.search(pattern, input_string)
    rv = list()
    if match:
        # Extract the numbers as a string
        numbers_str = match.group(1)

        # Convert the string of numbers into a list of floats
        rv = [float(num) for num in numbers_str.split(',')]

        return rv
    else:
        return None

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
    #
    # activateCommand = '1'
    # releaseCommand = '2'
    # resetCommand = '3'
    # ser.write(resetCommand.encode())


    # Create directories to save images and robot poses
    output_dir = 'calibration_data'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    image_dir = os.path.join(output_dir, 'images')
    if not os.path.exists(image_dir):
        os.makedirs(image_dir)

    poses_file = os.path.join(output_dir, 'robot_poses.txt')

    # Connect to the first available camera
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

    # Start grabbing continuously (video) with minimal delay
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    converter = pylon.ImageFormatConverter()

    # Convert to OpenCV BGR format
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    image_count = 0  # Counter for image filenames

    print("Press 'h' to capture an image and record the robot's pose, or 'q' to quit.")

    robotOrigin = [3.235471,633.705200,573.583679,179.734848,0.834654,-89.051025]
    robotMovement = [
        [100, 0, 0, 0, 0, 0],
        [0, 30, 0, 0, 0, 0],
        [100, 30, 0, 0, 0, 0],
        [150, 0, 0, 0, 0, 0],
        [0, 30, 0, 0, 0, 0],
        [150, 30, 0, 0, 0, 0],

        [100, 0, -100, 0, 0, 0],
        [0, 30, -100, 0, 0, 0],
        [100, 30, -100, 0, 0, 0],
        [150, 0, -100, 0, 0, 0],
        [0, 30, -100, 0, 0, 0],
        [150, 30, -100, 0, 0, 0],

        [100, 0, -150, 0, 0, 0],
        [0, 30, -150, 0, 0, 0],
        [100, 30, -150, 0, 0, 0],
        [150, 0, -150, 0, 0, 0],
        [0, 30, -150, 0, 0, 0],
        [150, 30, -150, 0, 0, 0],
        [0,0,0,0,0,0]
    ]
    robotMovementLength = len(robotMovement)
    robotMovementIndex = 0

    with open(poses_file, 'w') as f:
        while camera.IsGrabbing() and robotMovementIndex < robotMovementLength:
            # move the robot to the next position
            nextPoint = [x + y for x, y in zip(robotOrigin, robotMovement[robotMovementIndex])]
            RunPoint(move, nextPoint)
            WaitArrive(nextPoint)
            time.sleep(1)
            # grap the result
            grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            if grabResult.GrabSucceeded():
                # Access the image data
                image = converter.Convert(grabResult)
                img = image.GetArray()

                # Display the image
                cv2.namedWindow('Live Feed', cv2.WINDOW_NORMAL)
                cv2.imshow('Live Feed', img)

                key = cv2.waitKey(1) & 0xFF

                # Save the current image
                image_count += 1
                image_path = os.path.join(image_dir, f'calibration_{image_count:03d}.jpg')
                cv2.imwrite(image_path, img)
                print(f"Captured and saved {image_path}")

                # Get and save the robot's pose
                pose = robot_pose_parser(dashboard.GetPose())
                f.write(f"{image_path} {pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}\n")
                print(f"Recorded robot pose: {pose}")

                # increase the robot movement index
                robotMovementIndex += 1

            grabResult.Release()

    # Release the camera and close all windows
    camera.StopGrabbing()
    cv2.destroyAllWindows()

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
    # RunPoint(move, point_a)
    # WaitArrive(point_a)
