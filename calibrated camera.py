from pypylon import pylon
import cv2
import numpy as np

# Assuming you have already calibrated your camera and have the following:
# camera_matrix and dist_coefficients

# Example camera matrix and distortion coefficients
# These should be replaced with the actual calibration data
# camera_matrix = np.array([[fx, 0, cx],
#                           [0, fy, cy],
#                           [0,  0,  1]], dtype=np.float32)
#
# dist_coefficients = np.array([k1, k2, p1, p2, k3], dtype=np.float32)
camera_matrix = np.array([[2452.77, 0, 1846.52],
                          [0, 2453.79, 1397.71],
                          [0,  0,  1]], dtype=np.float32)
dist_coefficients = np.array([-0.24818, 0.13173, 0.00068, -0.00053, -0.04507], dtype=np.float32)

# Connecting to the first available camera
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

# Grabbing continuously (video) with minimal delay
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
converter = pylon.ImageFormatConverter()

# Converting to OpenCV BGR format
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grabResult.GrabSucceeded():
        # Access the image data
        image = converter.Convert(grabResult)
        img = image.GetArray()

        # Undistort the image
        undistorted_img = cv2.undistort(img, camera_matrix, dist_coefficients, None, camera_matrix)

        # Display the undistorted image
        cv2.namedWindow('Undistorted Image', cv2.WINDOW_NORMAL)
        cv2.imshow('Undistorted Image', undistorted_img)

        k = cv2.waitKey(1)
        if k == 27:  # Press 'Esc' to exit
            break

    grabResult.Release()

# Releasing the resource
camera.StopGrabbing()
cv2.destroyAllWindows()
