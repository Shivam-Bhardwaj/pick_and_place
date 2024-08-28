import cv2
import numpy as np
from pypylon import pylon

# Define the ArUco marker dictionary and size (in meters)
marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
marker_size = 0.05  # 50mm in meters

# Your camera intrinsic parameters (replace with actual calibration data)
camera_matrix = np.array([[2431.93, 0, 1843.19],
                          [0, 2431.40, 1395.97],
                          [0,  0,  1]], dtype=np.float32)

dist_coeffs = np.array([-0.24168, 0.11930, -0.00064, -0.00077, -0.02421], dtype=np.float32)

# Initialize the Basler camera
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

# Capture a single image
grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
if grab_result.GrabSucceeded():
    # Convert the image to OpenCV format
    image = converter.Convert(grab_result)
    img = image.GetArray()

    # Convert to grayscale as ArUco detection works on grayscale images
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers in the image
    aruco_params = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, marker_dict, parameters=aruco_params)

    # If markers are detected
    if ids is not None:
        # Estimate the pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        # List to hold the 3D positions of detected markers
        marker_positions = []

        for i in range(len(ids)):
            # Extract the pose (translation vector) of the marker
            tvec = tvecs[i][0]
            marker_positions.append(tvec)

            # Draw the detected marker and its ID on the image
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            cv2.aruco.drawAxis(img, camera_matrix, dist_coeffs, rvecs[i], tvec, 0.01)

        # Calculate distances between detected markers
        for i in range(len(marker_positions)):
            for j in range(i + 1, len(marker_positions)):
                distance = np.linalg.norm(marker_positions[i] - marker_positions[j])
                print(f"Distance between marker {ids[i][0]} and marker {ids[j][0]}: {distance:.4f} meters")

        # Display the image with detected markers and axes
        cv2.imshow('ArUco Marker Detection', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

# Release the camera
camera.StopGrabbing()
grab_result.Release()
