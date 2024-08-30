import cv2
import numpy as np
from pypylon import pylon

# Updated camera calibration data
camera_matrix = np.array([[2452.77, 0, 1846.52],
                          [0, 2453.79, 1397.71],
                          [0,  0,  1]], dtype=np.float32)

dist_coefficients = np.array([-0.24818, 0.13173, 0.00068, -0.00053, -0.04507], dtype=np.float32)

# Connecting to the first available camera
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

# Converter to OpenCV format
converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

# Define the ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

# Define the 3D points of the ArUco marker corners in the marker's coordinate system
marker_size = 0.05  # size of the marker (in meters)
marker_corners_3d = np.array([
    [-marker_size / 2, -marker_size / 2, 0],
    [ marker_size / 2, -marker_size / 2, 0],
    [ marker_size / 2,  marker_size / 2, 0],
    [-marker_size / 2,  marker_size / 2, 0]
], dtype=np.float32)

while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grabResult.GrabSucceeded():
        image = converter.Convert(grabResult)
        img = image.GetArray()

        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers in the image
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and len(ids) >= 4:  # Check if at least 4 markers are detected
            tvecs_dict = {}

            # Estimate pose for each detected ArUco marker
            for i in range(len(ids)):
                marker_id = ids[i][0]
                retval, rvec, tvec = cv2.solvePnP(marker_corners_3d, corners[i][0], camera_matrix, dist_coefficients)

                # Convert tvec from meters to millimeters and store it
                tvec_mm = tvec.flatten() * 1000
                tvecs_dict[marker_id] = tvec_mm

                # Draw the detected markers and their axes on the image
                cv2.drawFrameAxes(img, camera_matrix, dist_coefficients, rvec, tvec, 0.1)
                cv2.aruco.drawDetectedMarkers(img, corners, ids)

            if all(marker_id in tvecs_dict for marker_id in [0, 1, 2, 3]):
                # Compute the bounding box for the four markers
                markers_positions = np.array([corners[i][0] for i in range(len(ids))])
                bounding_rect = cv2.boundingRect(np.array(markers_positions).reshape(-1, 2))

                # Calculate the centroid of the markers
                markers_positions_world = np.array([tvecs_dict[0], tvecs_dict[1], tvecs_dict[2], tvecs_dict[3]])
                centroid = np.mean(markers_positions_world, axis=0)

                # Detect the black circle in the image
                # Apply a threshold to find dark areas
                _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

                # Debugging: Show thresholded image
                cv2.imshow('Thresholded Image', thresh)

                # Find contours
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    # Filter based on area to ignore small noise
                    area = cv2.contourArea(contour)
                    if area > 100:  # Adjust this threshold as needed
                        ((x, y), radius) = cv2.minEnclosingCircle(contour)

                        # Check if the circle is within the bounding box of the markers
                        if (bounding_rect[0] < x < bounding_rect[0] + bounding_rect[2] and
                                bounding_rect[1] < y < bounding_rect[1] + bounding_rect[3]):
                            # Draw the circle on the image
                            cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 0), 2)

                            # Convert circle center from pixel coordinates to world coordinates
                            circle_center_world = cv2.undistortPoints(
                                np.array([[[x, y]]], dtype=np.float32),
                                camera_matrix,
                                dist_coefficients
                            )
                            circle_center_world = circle_center_world.flatten() * 1000  # Convert to mm

                            # Calculate the relative position to the centroid
                            relative_position = circle_center_world - centroid[:2]  # Only x, y
                            print(f"Circle center relative to centroid in mm: {relative_position}")

                            # Draw the centroid and the circle center on the image
                            cv2.circle(img, (int(x), int(y)), 5, (0, 0, 255), -1)
                            cv2.putText(img, f"Relative Position: {relative_position[0]:.2f}, {relative_position[1]:.2f} mm",
                                        (int(x) + 10, int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                            break  # Exit the loop after finding the circle within the markers

        # Display the image with detected markers and circle
        cv2.namedWindow('Detected Markers and Circle', cv2.WINDOW_NORMAL)
        cv2.imshow('Detected Markers and Circle', img)

        k = cv2.waitKey(1)
        if k == 27:  # Press 'Esc' to exit
            break

    grabResult.Release()

camera.StopGrabbing()
cv2.destroyAllWindows()
