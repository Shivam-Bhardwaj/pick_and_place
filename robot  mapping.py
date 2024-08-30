import cv2
import numpy as np
from pypylon import pylon

# Camera calibration data (example values, replace with your actual calibration)
camera_matrix = np.array([[2431.93, 0, 1843.19],
                          [0, 2431.40, 1395.97],
                          [0,  0,  1]], dtype=np.float32)

dist_coefficients = np.array([-0.24168, 0.11930, -0.00064, -0.00077, -0.02421], dtype=np.float32)

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

        # Undistort the image
        undistorted_img = cv2.undistort(img, camera_matrix, dist_coefficients, None, camera_matrix)

        # Detect ArUco markers in the image
        gray = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and len(ids) >= 5:  # Check if at least 5 markers are detected (4 known + 1 new)
            # Draw the detected markers on the image
            cv2.aruco.drawDetectedMarkers(undistorted_img, corners, ids)

            # Dictionary to store the tvecs of detected markers
            tvecs_dict = {}

            # Process each detected marker
            for i in range(len(ids)):
                marker_id = ids[i][0]
                retval, rvec, tvec = cv2.solvePnP(marker_corners_3d, corners[i][0], camera_matrix, dist_coefficients)

                # Convert tvec from meters to inches
                tvec = tvec.flatten()

                # Store the tvec for each marker
                tvecs_dict[marker_id] = tvec

                # Draw axis using the cv2.drawFrameAxes function
                cv2.drawFrameAxes(undistorted_img, camera_matrix, dist_coefficients, rvec, tvec, 0.1)

            if all(marker_id in tvecs_dict for marker_id in [0, 1, 2, 3]):
                # Calculate the centroid of the four known markers
                markers_positions = np.array([tvecs_dict[0], tvecs_dict[1], tvecs_dict[2], tvecs_dict[3]])
                centroid = np.mean(markers_positions, axis=0)

                # Assume the new marker ID is the one not in [0, 1, 2, 3]
                new_marker_id = next(marker_id for marker_id in tvecs_dict if marker_id not in [0, 1, 2, 3])
                new_marker_position = tvecs_dict[new_marker_id]

                # Calculate the relative position of the new marker with respect to the centroid
                relative_position = new_marker_position - centroid

                print(f"Relative position of marker {new_marker_id} with respect to the centroid of the four markers: {relative_position}")

        # Display the image with detected markers
        cv2.namedWindow('Undistorted Image with ArUco Markers', cv2.WINDOW_NORMAL)
        cv2.imshow('Undistorted Image with ArUco Markers', undistorted_img)

        k = cv2.waitKey(1)
        if k == 27:  # Press 'Esc' to exit
            break

    grabResult.Release()

camera.StopGrabbing()
cv2.destroyAllWindows()
