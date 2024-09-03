import cv2
import cv2.aruco as aruco
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(4)  # Replace 4 with the correct index if needed

# Load the predefined dictionary for ArUco markers
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Real-world coordinates of ArUco markers (in millimeters)
aruco_positions = {
    0: (0.0, 0.0),
    1: (-685.8, 0.0),
    2: (-685.8, 330.2),
    3: (0.0, 330.2)
}

# Prepare to store the pixel coordinates of the ArUCo markers
pixel_positions = {}

# Hardcoded parameters
gaussian_kernel = 31  # Ensure kernel size is odd
threshold_value = 92
canny_min = 141
canny_max = 336
circularity_min = 35 / 100.0  # Convert to 0.6
circularity_max = 164 / 100.0  # Convert to 1.64
aspect_ratio_min = 96 / 100.0  # Convert to 0.96
aspect_ratio_max = 159 / 100.0  # Convert to 1.59

if not cap.isOpened():
    print("Error: Could not open video stream from webcam.")
    exit()

while True:
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

                shapes_info = []

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

                shapes_info.sort(key=lambda c: c['area'])

                for i, shape in enumerate(shapes_info):
                    if shape['shape'] == 'Circle':
                        cv2.circle(frame, shape['pixel_center'], int(shape['radius']), (255, 0, 0), 2)
                    elif shape['shape'] in ['Square', 'Rectangle']:
                        cv2.drawContours(frame, [shape['box']], 0, (0, 255, 255), 2)
                        # Draw the orientation arrow
                        cv2.arrowedLine(frame, shape['pixel_center'], shape['arrow_end'], (0, 255, 0), 2, tipLength=0.3)
                        # Display orientation and rotation direction
                        cv2.putText(frame, f"Angle: {shape['orientation']:.2f}",
                                    (shape['pixel_center'][0] + 10, shape['pixel_center'][1] + 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.circle(frame, shape['pixel_center'], 5, (0, 255, 0), -1)
                    cv2.putText(frame, f"{shape['shape']} Rank: {i+1}",
                                (shape['pixel_center'][0] + 10, shape['pixel_center'][1] - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, f"({shape['world_center'][0]:.2f}, {shape['world_center'][1]:.2f})",
                                (shape['pixel_center'][0] + 10, shape['pixel_center'][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Shape Detection with Area Ranking and Real-World Coordinates", frame)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    except BaseException as e:
        print(f"An error occurred: {e}")
        break

# Release the capture and close theHere is the continuation of the optimized code:
cap.release()
cv2.destroyAllWindows()
