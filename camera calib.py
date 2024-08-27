import cv2
import numpy as np
import glob

# Define the dimensions of the checkerboard
CHECKERBOARD = (9, 6)  # Change this to match your checkerboard dimensions (number of corners per row/col)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Create the 3D object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images
objpoints = []  # 3D point in real world space
imgpoints = []  # 2D points in image plane

# Load images from the directory
images = glob.glob('calibration_images/*.jpg')  # Replace with the path to your calibration images

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print out the results in the desired format
print("camera_matrix = np.array([[{:.2f}, 0, {:.2f}],".format(camera_matrix[0, 0], camera_matrix[0, 2]))
print("                          [0, {:.2f}, {:.2f}],".format(camera_matrix[1, 1], camera_matrix[1, 2]))
print("                          [0,  0,  1]], dtype=np.float32)\n")

print("dist_coefficients = np.array([{:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}], dtype=np.float32)".format(
    dist_coeffs[0, 0], dist_coeffs[0, 1], dist_coeffs[0, 2], dist_coeffs[0, 3], dist_coeffs[0, 4]))
