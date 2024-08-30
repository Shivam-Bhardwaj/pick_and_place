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

# Adjust detector parameters for faster processing
parameters = cv2.aruco.DetectorParameters()
parameters.minMarkerPerimeterRate = 0.03
parameters.maxMarkerPerimeterRate = 4.0

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

        # Downscale the image to speed up processing
        small_img = cv2.resize(img, (640, 480))

        # Convert to grayscale
        gray = cv2.cvtColor(small_img, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers in the downscaled image
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and len(ids) >= 4:  # Check if at least 4 markers are detected
            tvecs_dict = {}

            for i in range(len(ids)):
                marker_id = ids[i][0]
                retval, rvec, tvec = cv2.solvePnP(marker_corners_3d, corners[i][0], camera_matrix, dist_coefficients)

                # Convert tvec from meters to millimeters and store it
                tvec_mm = tvec.flatten() * 1000
                tvecs_dict[marker_id] = tvec_mm

            if all(marker_id in tvecs_dict for marker_id in [0, 1, 2, 3]):
                markers_positions = np.array([tvecs_dict[0], tvecs_dict[1], tvecs_dict[2], tvecs_dict[3]])
                centroid = np.mean(markers_positions, axis=0)
                print(f"Centroid of the markers (0, 1, 2, 3) in mm: {centroid}")

    grabResult.Release()

camera.StopGrabbing()
