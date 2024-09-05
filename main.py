import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re

import serial
import time

import cv2
import cv2.aruco as aruco
import numpy as np

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
    
    

    # arduinoPort = "/dev/ttyACM0"
    # baudRate = 115200
    # timeout = 1
    # ser = serial.Serial(arduinoPort, baudRate, timeout=timeout)
    # time.sleep(2)

    # activateCommand = '2'
    # releaseCommand = '1'
    # resetCommand = '3'
    # ser.write(resetCommand.encode())
    
    cap = cv2.VideoCapture(4)  # Replace 4 with the correct index if needed

    # Load the predefined dictionary for ArUco markers
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    # Real-world coordinates of ArUco markers (in millimeters)
    aruco_positions = {
        0: (0.0, 0.0),
        1: (-711.2, 0.0),
        2: (-711.2, 355.6),
        3: (0.0, 355.6)
    }

    # Prepare to store the pixel coordinates of the ArUCo markers
    pixel_positions = {}

    # Hardcoded parameters
    gaussian_kernel = 31  # Ensure kernel size is odd
    threshold_value = 151
    canny_min = 141
    canny_max = 336
    circularity_min = 60 / 100.0  # Convert to 0.6
    circularity_max = 164 / 100.0  # Convert to 1.64
    aspect_ratio_min = 96 / 100.0  # Convert to 0.96
    aspect_ratio_max = 159 / 100.0  # Convert to 1.59

    if not cap.isOpened():
        print("Error: Could not open video stream from webcam.")
        exit()
    
    robot_zero_position = [37.545567,523.508423,643.673096,179.513641,0.670840,177.957626]
    workspace_zero_robot_position = [372.023956,396.518036,643.673096,179.513641,0.670840,177.957626]
    RunPoint(move, robot_zero_position)
    WaitArrive(robot_zero_position)
    dashboard.GetPose()
    print("Start analyzing the images...")
    shapes_info = []
    for i in range(10):
        shapes_info = []
        try:
            # Capture frame-by-frame
            ret, frame = cap.read()

            if not ret:
                print("Error: Failed to capture image.")
                break

            # Convert the frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers in the frame
            corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # Create a mask to exclude ArUco markers from the shape detection
            mask = np.ones(gray.shape, dtype=np.uint8) * 255  # Start with a white mask

            if ids is not None:
                for i in range(len(ids)):
                    corner = corners[i][0]
                    int_corners = np.int32(corner)  # Convert corners to integer
                    cv2.fillPoly(mask, [int_corners], 0)  # Mask out the ArUco marker area

                    cX = int(np.mean(corner[:, 0]))
                    cY = int(np.mean(corner[:, 1]))
                    pixel_positions[ids[i][0]] = (cX, cY)

                    cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                    cv2.putText(frame, f"ID: {ids[i][0]}", (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                if len(pixel_positions) == 4:  # Ensure all markers are detected
                    src_points = np.array([pixel_positions[0], pixel_positions[1], pixel_positions[2], pixel_positions[3]], dtype="float32")
                    dst_points = np.array([aruco_positions[0], aruco_positions[1], aruco_positions[2], aruco_positions[3]], dtype="float32")

                    matrix = cv2.getPerspectiveTransform(src_points, dst_points)

                    masked_gray = cv2.bitwise_and(gray, gray, mask=mask)

                    blur_value = cv2.GaussianBlur(masked_gray, (gaussian_kernel, gaussian_kernel), 0)

                    _, thresh = cv2.threshold(blur_value, threshold_value, 255, cv2.THRESH_BINARY_INV)

                    thresh = cv2.bitwise_and(thresh, thresh, mask=mask)

                    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    

                    for contour in contours:
                        approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)

                        if len(approx) > 4:
                            area = cv2.contourArea(contour)
                            (x, y), radius = cv2.minEnclosingCircle(contour)
                            circularity = area / (np.pi * (radius ** 2))
                            if circularity_min <= circularity <= circularity_max:
                                circle_center_img = np.array([[x, y]], dtype="float32")
                                circle_center_world = cv2.perspectiveTransform(np.array([circle_center_img]), matrix)
                                circle_center_world = circle_center_world[0][0]

                                shapes_info.append({
                                    'shape': 'Circle',
                                    'area': area,
                                    'radius': radius,
                                    'pixel_center': (int(x), int(y)),
                                    'world_center': (circle_center_world[0], circle_center_world[1])
                                })

                        elif len(approx) == 4:
                            rect = cv2.minAreaRect(contour)
                            box = cv2.boxPoints(rect)
                            box = np.int32(box)  # Updated to avoid the np.int0 error

                            width = rect[1][0]
                            height = rect[1][1]
                            aspect_ratio = width / float(height) if height != 0 else 0

                            if aspect_ratio_min <= aspect_ratio <= aspect_ratio_max:
                                shape_type = "Square" if 0.9 <= aspect_ratio <= 1.1 else "Rectangle"
                                rect_center_img = np.array([[rect[0][0], rect[0][1]]], dtype="float32")
                                rect_center_world = cv2.perspectiveTransform(np.array([rect_center_img]), matrix)
                                rect_center_world = rect_center_world[0][0]

                                # Directly use the angle provided by minAreaRect
                                orientation_to_x_axis = rect[2]

                                # Adjust for y-axis comparison
                                if orientation_to_x_axis < -45:
                                    orientation_to_x_axis = 90 + orientation_to_x_axis  # Compensate for OpenCV's angle definition

                                # Determine rotation direction
                                if orientation_to_x_axis >= 0:
                                    rotation_direction = "Counter-Clockwise"
                                else:
                                    rotation_direction = "Clockwise"

                                # Calculate the end point of the arrow to show the orientation
                                arrow_length = 50  # Length of the arrow in pixels
                                end_point = (
                                    int(rect[0][0] + arrow_length * np.cos(np.radians(orientation_to_x_axis))),
                                    int(rect[0][1] + arrow_length * np.sin(np.radians(orientation_to_x_axis)))
                                )

                                shapes_info.append({
                                    'shape': shape_type,
                                    'area': width * height,
                                    'pixel_center': (int(rect[0][0]), int(rect[0][1])),
                                    'world_center': (rect_center_world[0], rect_center_world[1]),
                                    'box': box,
                                    'orientation': orientation_to_x_axis,
                                    'rotation_direction': rotation_direction,
                                    'arrow_end': end_point
                                })

                    shapes_info.sort(key=lambda c: c['area'], reverse=True)
                    

                    # for i, shape in enumerate(shapes_info):
                    #     if shape['shape'] == 'Circle':
                    #         cv2.circle(frame, shape['pixel_center'], int(shape['radius']), (255, 0, 0), 2)
                    #     elif shape['shape'] in ['Square', 'Rectangle']:
                    #         cv2.drawContours(frame, [shape['box']], 0, (0, 255, 255), 2)
                    #         # Draw the orientation arrow
                    #         cv2.arrowedLine(frame, shape['pixel_center'], shape['arrow_end'], (0, 255, 0), 2, tipLength=0.3)
                    #         # Display orientation and rotation direction
                    #         cv2.putText(frame, f"Angle: {shape['orientation']:.2f}",
                    #                     (shape['pixel_center'][0] + 10, shape['pixel_center'][1] + 30),
                    #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    #     cv2.circle(frame, shape['pixel_center'], 5, (0, 255, 0), -1)
                    #     cv2.putText(frame, f"{shape['shape']} Rank: {i+1}",
                    #                 (shape['pixel_center'][0] + 10, shape['pixel_center'][1] - 30),
                    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    #     cv2.putText(frame, f"({shape['world_center'][0]:.2f}, {shape['world_center'][1]:.2f})",
                    #                 (shape['pixel_center'][0] + 10, shape['pixel_center'][1] - 10),
                    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)                   

            # cv2.imshow("Shape Detection with Area Ranking and Real-World Coordinates", frame)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
        except BaseException as e:
            print(f"An error occurred: {e}")
            break

    # Release the capture and close theHere is the continuation of the optimized code:
    cap.release()
    cv2.destroyAllWindows()
    print("Analyzation done!")
    
    print(f"Start moving robot for {len(shapes_info)} objects")
    square_drop_top = [-411.482391,-229.931000,643.298584,179.513504,0.671409,-51.681812]
    circle_drop_top = [-460.979065,98.408943,643.298584,179.513504,0.671409,-92.928215]
    
    square_drop_offset = square_drop_top.copy() 
    square_drop_offset[2] = 200
    
    circle_drop_offset = circle_drop_top.copy()
    circle_drop_offset[2] = 200
    
    circle_drop = circle_drop_top.copy()
    circle_drop[2] = 60.058582
    
    square_drop = square_drop_top.copy()
    square_drop[2] = 58.098583
     
    
    square_drop_thickness = 7
    circle_drop_thickness = 7
    
    
    for workpeice in shapes_info: 
        # calculate the workpiece position according to robot coordinate system
        workpiece_robot_position_offset = workspace_zero_robot_position.copy()
        workpiece_workspace_pos =  workpeice['world_center']
        print(workpiece_workspace_pos)
        workpiece_robot_position_offset[0] += workpiece_workspace_pos[0]
        workpiece_robot_position_offset[1] += workpiece_workspace_pos[1]
        workpiece_robot_position_offset[2] = 200
        
        
        workpiece_robot_position = workpiece_robot_position_offset.copy()
        workpiece_robot_position[2] = 75
        
        # move to the robot origin position
        RunPoint(move, robot_zero_position)
        WaitArrive(robot_zero_position)
        
        # move the robot to the workpiece with offset
        RunPoint(move, workpiece_robot_position_offset)
        WaitArrive(workpiece_robot_position_offset)
        
        # turn on sunction 
        # ser.write(activateCommand.encode())
        # time.sleep(1)
        # move to the workpeice
        RunPoint(move, workpiece_robot_position)
        WaitArrive(workpiece_robot_position)
        
        # move the robot up 
        RunPoint(move, workpiece_robot_position_offset)
        WaitArrive(workpiece_robot_position_offset)
        
        # move back to robot origin
        RunPoint(move, robot_zero_position)
        WaitArrive(robot_zero_position)
        
        
        # check whether it's a square or not
        if workpeice['shape'] == 'Circle': 
            # move to circle top
            RunPoint(move, circle_drop_top)
            WaitArrive(circle_drop_top)
            
            # move to circle offset 
            RunPoint(move, circle_drop_offset)
            WaitArrive(circle_drop_offset)
            
            # move to drop point
            RunPoint(move, circle_drop)
            WaitArrive(circle_drop)
            
            # realease sunction
            # ser.write(releaseCommand.encode())
            # time.sleep(1)
            
            # update next circle drop position
            circle_drop[2] += circle_drop_thickness
            
            # move back to circle top
            RunPoint(move, circle_drop_top)
            # WaitArrive(circle_drop_top)
            
        else: # move to square position
            # move to square top
            RunPoint(move, square_drop_top)
            WaitArrive(square_drop_top)
            
            # move to square offset 
            RunPoint(move, square_drop_offset)
            WaitArrive(square_drop_offset)
            
            # get the orientation and rotate it
            angle = workpeice['orientation']
            square_drop[5] -= angle
            
            # move to drop point
            RunPoint(move, square_drop)
            WaitArrive(square_drop)
            
            # rotate it back
            square_drop[5] += angle
            
            # realease sunction
            # ser.write(releaseCommand.encode())
            # time.sleep(1)
            
            # update next square drop position
            square_drop[2] += square_drop_thickness
            
            # move back to square top
            RunPoint(move, square_drop_top)
            WaitArrive(square_drop_top)
              
    RunPoint(move, robot_zero_position)
    WaitArrive(robot_zero_position)     