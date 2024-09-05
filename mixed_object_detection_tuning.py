import cv2
import cv2.aruco as aruco
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(2)  # Replace 4 with the correct index if needed

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

# Callback function for trackbars (required but unused)
def nothing(x):
    pass

# Create a window for the GUI
cv2.namedWindow('Shape Detection')

# Create trackbars for adjusting parameters with default values
cv2.createTrackbar('Gaussian Kernel', 'Shape Detection', 50, 50, nothing)
cv2.createTrackbar('Threshold', 'Shape Detection', 141, 255, nothing)
cv2.createTrackbar('Canny Min', 'Shape Detection', 141, 500, nothing)
cv2.createTrackbar('Canny Max', 'Shape Detection', 336, 500, nothing)
cv2.createTrackbar('Circularity Min', 'Shape Detection', 60, 100, nothing)  # Default 60%
cv2.createTrackbar('Circularity Max', 'Shape Detection', 164, 200, nothing)  # Default 164%
cv2.createTrackbar('Aspect Ratio Min', 'Shape Detection', 96, 200, nothing)  # Default 96%
cv2.createTrackbar('Aspect Ratio Max', 'Shape Detection', 159, 200, nothing)  # Default 159%

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

                # Get current positions of trackbars
                ksize = cv2.getTrackbarPos('Gaussian Kernel', 'Shape Detection')
                if ksize % 2 == 0:  # Ensure kernel size is odd
                    ksize += 1

                blur_value = cv2.GaussianBlur(masked_gray, (ksize, ksize), 0)

                threshold_value = cv2.getTrackbarPos('Threshold', 'Shape Detection')
                _, thresh = cv2.threshold(blur_value, threshold_value, 255, cv2.THRESH_BINARY_INV)

                thresh = cv2.bitwise_and(thresh, thresh, mask=mask)

                contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                shapes_info = []

                # Get circularity and aspect ratio limits from trackbars
                circularity_min = cv2.getTrackbarPos('Circularity Min', 'Shape Detection') / 100.0
                circularity_max = cv2.getTrackbarPos('Circularity Max', 'Shape Detection') / 100.0
                aspect_ratio_min = cv2.getTrackbarPos('Aspect Ratio Min', 'Shape Detection') / 100.0
                aspect_ratio_max = cv2.getTrackbarPos('Aspect Ratio Max', 'Shape Detection') / 100.0

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

                            shapes_info.append({
                                'shape': shape_type,
                                'area': width * height,
                                'pixel_center': (int(rect[0][0]), int(rect[0][1])),
                                'world_center': (rect_center_world[0], rect_center_world[1]),
                                'box': box
                            })

                shapes_info.sort(key=lambda c: c['area'], reverse=True)

                for i, shape in enumerate(shapes_info):
                    if shape['shape'] == 'Circle':
                        cv2.circle(frame, shape['pixel_center'], int(shape['radius']), (255, 0, 0), 2)
                    elif shape['shape'] in ['Square', 'Rectangle']:
                        cv2.drawContours(frame, [shape['box']], 0, (0, 255, 255), 2)
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
        cap.release()
        cv2.destroyAllWindows()
        raise e

cap.release()
cv2.destroyAllWindows()
''