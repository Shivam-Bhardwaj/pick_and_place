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
gaussian_kernel = 50
threshold_value = 141
canny_min = 141
canny_max = 336
circularity_min = 60 / 100.0  # Convert to 0.6
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

                if gaussian_kernel % 2 == 0:  # Ensure kernel size is odd
                    gaussian_kernel += 1

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

                            # Calculate orientation relative to the positive y-axis (Aruco 0 to Aruco 3)
                            angle = rect[2]  # Angle of the rectangle in the image
                            delta_x = rect_center_world[0] - aruco_positions[0][0]
                            delta_y = rect_center_world[1] - aruco_positions[0][1]
                            orientation = np.degrees(np.arctan2(delta_y, delta_x))

                            # Determine the relative orientation to the y-axis (Aruco 0 to Aruco 3)
                            angle_to_y_axis = np.abs(orientation - 90)
                            if angle_to_y_axis > 90:
                                rotation_direction = "Clockwise"
                            else:
                                rotation_direction = "Counter-Clockwise"

                            # Calculate arrow direction
                            arrow_length = 50  # Adjust as needed
                            angle_radians = np.radians(angle)
                            end_x = int(rect[0][0] + arrow_length * np.cos(angle_radians))
                            end_y = int(rect[0][1] + arrow_length * np.sin(angle_radians))

                            shapes_info.append({
                                'shape': shape_type,
                                'area': width * height,
                                'pixel_center': (int(rect[0][0]), int(rect[0][1])),
                                'world_center': (rect_center_world[0], rect_center_world[1]),
                                'box': box,
                                'orientation': angle_to_y_axis,
                                'rotation_direction': rotation_direction,
                                'arrow_end': (end_x, end_y)
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
                        # angle_text = f"Angle: {shape['orientation']:.2f} {shape['rotation_direction']}"
                        angle_text = f"Angle: {shape['orientation']:.2f}"
                        
                        cv2.putText(frame, angle_text,
                                    (shape['pixel_center'][0] + 10, shape['pixel_center'][1] + 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.circle(frame, shape['pixel_center'], 5, (0, 255, 0), -1)
                    cv2.putText(frame, f"{shape['shape']} Rank: {i+1}",
                                (shape['pixel_center'][0] + 10, shape['pixel_center'][1] - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, f"({shape['world_center'][0]:.2f}, {shape['world_center'][1]:.2f})",
                                (shape['pixel_center'][0] + 10, shape['pixel_center'][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Shape Detection with Orientation Arrows", frame)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    except BaseException as e:
        cap.release()
        cv2.destroyAllWindows()
        raise e

cap.release()
cv2.destroyAllWindows()
