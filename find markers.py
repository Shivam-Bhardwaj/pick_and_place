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

# Define the AprilTag detector
detector = cv2.aruco.ArucoDetector()
tag_size = 0.05  # size of the AprilTag (in meters)

# Define the 3D points of the AprilTag corners in the tag's coordinate system
# Assuming the tag is square and centered at the origin
width = float(27 * 25.4) # mm 
height = float(13 * 25.4) # mm
tag_corners_3d = np.array([
    [-tag_size / 2, -tag_size / 2, 0],
    [ tag_size / 2, -tag_size / 2, 0],
    [ tag_size / 2,  tag_size / 2, 0],
    [-tag_size / 2,  tag_size / 2, 0]
], dtype=np.float32)

while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grabResult.GrabSucceeded():
        image = converter.Convert(grabResult)
        img = image.GetArray()

        # Undistort the image
        undistorted_img = cv2.undistort(img, camera_matrix, dist_coefficients, None, camera_matrix)

        # Detect AprilTags in the image
        gray = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            # Draw the detected tags on the image
            cv2.aruco.drawDetectedMarkers(undistorted_img, corners, ids)

            # Estimate pose for each detected AprilTag
            for i in range(len(ids)):
                # Use solvePnP to estimate pose
                retval, rvec, tvec = cv2.solvePnP(tag_corners_3d, corners[i][0], camera_matrix, dist_coefficients)
                
                # Draw axis using the cv2.drawFrameAxes function
                cv2.drawFrameAxes(undistorted_img, camera_matrix, dist_coefficients, rvec, tvec, 0.1)
                
                print(f"Tag ID: {ids[i][0]} - Translation vector: {tvec.T} - Rotation vector: {rvec.T}")

        # Display the image with detected tags
        cv2.namedWindow('Undistorted Image with AprilTags', cv2.WINDOW_NORMAL)
        cv2.imshow('Undistorted Image with AprilTags', undistorted_img)

        k = cv2.waitKey(1)
        if k == 27:  # Press 'Esc' to exit
            break

    grabResult.Release()

camera.StopGrabbing()
cv2.destroyAllWindows()
