import cv2
import numpy as np

# Load the captured robot poses and image paths
poses_file = 'calibration_data/robot_poses.txt'
image_paths = '../calibration_data/images'
robot_poses = []

with open(poses_file, 'r') as f:
    for line in f.readlines():
        parts = line.strip().split()
        image_paths.append(parts[0])
        robot_poses.append([float(val) for val in parts[1:]])

# Define the checkerboard dimensions
CHECKERBOARD = (9, 6)  # Adjust based on your checkerboard

# Prepare object points based on the checkerboard size
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []  # 3D points in the real world space
imgpoints = []  # 2D points in the image plane

# Load the camera matrix and distortion coefficients
# You should replace these with your actual calibration data
camera_matrix = np.array([[2431.93, 0, 1843.19],
                          [0, 2431.40, 1395.97],
                          [0,  0,  1]], dtype=np.float32)

dist_coefficients = np.array([-0.24168, 0.11930, -0.00064, -0.00077, -0.02421], dtype=np.float32)

# Process each image and pose
for image_path, pose in zip(image_paths, robot_poses):
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), None)
        imgpoints.append(corners2)

# Perform hand-eye calibration
ret, rvecs, tvecs, inliers = cv2.solvePnPRansac(np.array(objpoints), np.array(imgpoints), camera_matrix, dist_coefficients)

# Extract the rotation and translation matrices
rotation_matrix, _ = cv2.Rodrigues(rvecs)
translation_vector = tvecs

print("Rotation Matrix (R):\n", rotation_matrix)
print("Translation Vector (T):\n", translation_vector)
