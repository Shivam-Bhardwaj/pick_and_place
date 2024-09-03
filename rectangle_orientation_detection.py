import cv2
import cv2.aruco as aruco
import numpy as np
import math

# Initialize the webcam
cap = cv2.VideoCapture(4)  # Replace 4 with the correct index if needed

# Load the predefined dictionary for ArUco markers
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Real-world coordinates of ArUco markers (in millimeters)
aruco_positions = {
    0: (0.0, 0.0),
    1: (-685.8, 0.0),       # Negative x-axis direction
    2: (-685.8, 330.2),      # Coordinates in millimeters
    3: (0.0, 330.2)          # Positive y-axis direction
}

# Prepare to store the pixel coordinates of the ArUco markers
pixel_positions = {}

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
            # For each detected ArUco marker, store its pixel coordinates and update the mask
            for i in range(len(ids)):
                corner = corners[i][0]
                int_corners = np.int32(corner)  # Convert corners to integer
                cv2.fillPoly(mask, [int_corners], 0)  # Mask out the ArUco marker area

                # Calculate the center of the ArUco marker in pixels and mark it
                cX = int(np.mean(corner[:, 0]))
                cY = int(np.mean(corner[:, 1]))
                pixel_positions[ids[i][0]] = (cX, cY)

            if len(pixel_positions) == 4:  # Ensure all markers are detected
                # Now that you have both pixel and real-world positions, calculate the transformation

                # Extract points for cv2.getPerspectiveTransform
                src_points = np.array([pixel_positions[0], pixel_positions[1], pixel_positions[2], pixel_positions[3]], dtype="float32")
                dst_points = np.array([aruco_positions[0], aruco_positions[1], aruco_positions[2], aruco_positions[3]], dtype="float32")

                # Calculate the perspective transform matrix
                matrix = cv2.getPerspectiveTransform(src_points, dst_points)

                # Apply the mask to the original grayscale image
                masked_gray = cv2.bitwise_and(gray, gray, mask=mask)

                # Apply a Gaussian blur to reduce noise and improve detection accuracy
                blurred = cv2.GaussianBlur(masked_gray, (5, 5), 0)

                # Threshold the image to get a binary image
                _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)

                # Apply the mask to the thresholded image
                thresh = cv2.bitwise_and(thresh, thresh, mask=mask)

                # Find contours in the masked thresholded image
                contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    # Approximate the contour to find shapes
                    approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)

                    # Detect rectangles (excluding ArUco areas)
                    if len(approx) == 4:
                        x, y, w, h = cv2.boundingRect(approx)
                        aspect_ratio = w / float(h)
                        if 0.9 <= aspect_ratio <= 1.1:  # This ensures it's close to a square
                            shape_type = "Square"
                        else:
                            shape_type = "Rectangle"

                        # Calculate the orientation of the rectangle relative to the axes
                        p1, p2 = approx[0][0], approx[1][0]  # Take the first two points of the rectangle
                        angle = math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))

                        # Convert the pixel center to real-world coordinates
                        rect_center_img = np.array([[x + w / 2, y + h / 2]], dtype="float32")
                        rect_center_world = cv2.perspectiveTransform(np.array([rect_center_img]), matrix)
                        rect_center_world = rect_center_world[0][0]

                        # Draw the rectangle
                        cv2.rectangle(frame,
                                      (x, y),
                                      (x + w, y + h),
                                      (0, 255, 255), 2)

                        # Draw the orientation line
                        p1 = (int(x + w / 2), int(y + h / 2))
                        length = int(w / 2)
                        p2 = (int(p1[0] + length * math.cos(math.radians(angle))),
                              int(p1[1] + length * math.sin(math.radians(angle))))
                        cv2.arrowedLine(frame, p1, p2, (255, 0, 255), 2, tipLength=0.3)

                        # Draw the center of the rectangle
                        cv2.circle(frame, p1, 5, (0, 255, 0), -1)

                        # Display the real-world coordinates (without units) and orientation angle
                        cv2.putText(frame, f"{shape_type}",
                                    (p1[0] + 10, p1[1] - 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(frame, f"({rect_center_world[0]:.2f}, {rect_center_world[1]:.2f})",
                                    (p1[0] + 10, p1[1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(frame, f"Angle: {angle:.2f} deg",
                                    (p1[0] + 10, p1[1] + 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the frame with detected rectangles and their orientations
        cv2.imshow("Rectangle Detection with Orientation", frame)

        # Add a small delay to reduce CPU usage and allow for quitting
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    except BaseException as e:
        # Release the capture and close the windows
        cap.release()
        cv2.destroyAllWindows()
        raise e

# Release the capture and close the windows
cap.release()
cv2.destroyAllWindows()
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
    1: (-685.8, 0.0),       # Converted to millimeters
    2: (-685.8, 330.2),      # Converted to millimeters
    3: (0.0, 330.2)          # Converted to millimeters
}

# Prepare to store the pixel coordinates of the ArUCo markers
pixel_positions = {}

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
            # For each detected ArUco marker, store its pixel coordinates and update the mask
            for i in range(len(ids)):
                corner = corners[i][0]
                int_corners = np.int32(corner)  # Convert corners to integer
                cv2.fillPoly(mask, [int_corners], 0)  # Mask out the ArUco marker area

                # Calculate the center of the ArUco marker in pixels and mark it
                cX = int(np.mean(corner[:, 0]))
                cY = int(np.mean(corner[:, 1]))
                pixel_positions[ids[i][0]] = (cX, cY)

                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"ID: {ids[i][0]}", (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            if len(pixel_positions) == 4:  # Ensure all markers are detected
                # Now that you have both pixel and real-world positions, calculate the transformation

                # Extract points for cv2.getPerspectiveTransform
                src_points = np.array([pixel_positions[0], pixel_positions[1], pixel_positions[2], pixel_positions[3]], dtype="float32")
                dst_points = np.array([aruco_positions[0], aruco_positions[1], aruco_positions[2], aruco_positions[3]], dtype="float32")

                # Calculate the perspective transform matrix
                matrix = cv2.getPerspectiveTransform(src_points, dst_points)

                # Apply the mask to the original grayscale image
                masked_gray = cv2.bitwise_and(gray, gray, mask=mask)

                # Apply a Gaussian blur to reduce noise and improve detection accuracy
                blurred = cv2.GaussianBlur(masked_gray, (5, 5), 0)

                # Threshold the image to get a binary image
                _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)

                # Apply the mask to the thresholded image
                thresh = cv2.bitwise_and(thresh, thresh, mask=mask)

                # Find contours in the masked thresholded image
                contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                shapes_info = []

                for contour in contours:
                    # Approximate the contour to find shapes
                    approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)

                    # Detect circles
                    if len(approx) > 4:
                        area = cv2.contourArea(contour)
                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        circularity = area / (np.pi * (radius ** 2))
                        if 0.7 <= circularity <= 1.3:  # Circularity check
                            # Convert the pixel center to real-world coordinates
                            circle_center_img = np.array([[x, y]], dtype="float32")
                            circle_center_world = cv2.perspectiveTransform(np.array([circle_center_img]), matrix)
                            circle_center_world = circle_center_world[0][0]

                            # Store the circle's information
                            shapes_info.append({
                                'shape': 'Circle',
                                'area': area,
                                'radius': radius,
                                'pixel_center': (int(x), int(y)),
                                'world_center': (circle_center_world[0], circle_center_world[1])  # In millimeters
                            })

                    # Detect rectangles (excluding ArUco areas)
                    elif len(approx) == 4:
                        x, y, w, h = cv2.boundingRect(approx)
                        aspect_ratio = w / float(h)
                        if 0.9 <= aspect_ratio <= 1.1:  # This ensures it's close to a square
                            shape_type = "Square"
                        else:
                            shape_type = "Rectangle"
                        # Convert the pixel center to real-world coordinates
                        rect_center_img = np.array([[x + w / 2, y + h / 2]], dtype="float32")
                        rect_center_world = cv2.perspectiveTransform(np.array([rect_center_img]), matrix)
                        rect_center_world = rect_center_world[0][0]

                        # Store the rectangle's information
                        shapes_info.append({
                            'shape': shape_type,
                            'area': w * h,
                            'pixel_center': (int(x + w / 2), int(y + h / 2)),
                            'world_center': (rect_center_world[0], rect_center_world[1])  # In millimeters
                        })

                # Sort the shapes by area from smallest to largest
                shapes_info.sort(key=lambda c: c['area'])

                # Display the shapes with their area ranks and real-world coordinates
                for i, shape in enumerate(shapes_info):
                    # Draw the shape
                    if shape['shape'] == 'Circle':
                        cv2.circle(frame, shape['pixel_center'], int(shape['radius']), (255, 0, 0), 2)
                    elif shape['shape'] in ['Square', 'Rectangle']:
                        cv2.rectangle(frame,
                                      (shape['pixel_center'][0] - int(shape['area'] ** 0.5) // 2, shape['pixel_center'][1] - int(shape['area'] ** 0.5) // 2),
                                      (shape['pixel_center'][0] + int(shape['area'] ** 0.5) // 2, shape['pixel_center'][1] + int(shape['area'] ** 0.5) // 2),
                                      (0, 255, 255), 2)
                    # Draw the center of the shape
                    cv2.circle(frame, shape['pixel_center'], 5, (0, 255, 0), -1)
                    # Display the area ranking and real-world coordinates (without units)
                    cv2.putText(frame, f"{shape['shape']} Rank: {i+1}",
                                (shape['pixel_center'][0] + 10, shape['pixel_center'][1] - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, f"({shape['world_center'][0]:.2f}, {shape['world_center'][1]:.2f})",
                                (shape['pixel_center'][0] + 10, shape['pixel_center'][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the frame with detected shapes, area ranks, and coordinates
        cv2.imshow("Shape Detection with Area Ranking and Real-World Coordinates", frame)

        # Add a small delay to reduce CPU usage and allow for quitting
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    except BaseException as e:
        # Release the capture and close the windows
        cap.release()
        cv2.destroyAllWindows()
        raise e

# Release the capture and close the windows
cap.release()
cv2.destroyAllWindows()
