import cv2
import numpy as np

# Load the captured robot poses and image paths
poses_file = 'calibration_data/robot_poses.txt'
image_paths = []
robot_poses = []

def read_data(): 
    with open(poses_file, 'r') as f:
        # parse the read line
        for line in f.readlines():
            parts = line.strip().split()
            image_paths.append(parts[0])
            robot_poses.append([float(val) for val in parts[1:]])

def calculate_transformation(): 
    # Define the checkerboard dimensions
    CHECKERBOARD = (9, 6)  # Adjust based on your checkerboard

    # Prepare object points based on the checkerboard size
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    # Arrays to store object points and image points
    objpoints = []  # 3D points in the real world space
    imgpoints = []  # 2D points in the image plane

    # Load the camera matrix and distortion coefficients
    camera_matrix = np.array([[2431.93, 0, 1843.19],
                            [0, 2431.40, 1395.97],
                            [0,  0,  1]], dtype=np.float32)

    dist_coefficients = np.array([-0.24168, 0.11930, -0.00064, -0.00077, -0.02421], dtype=np.float32)

    # Process each image and pose
    for image_path, pose in zip(image_paths, robot_poses):
        img = cv2.imread(image_path)
        # Display the image in a window
        cv2.imshow('Image Window', img)

        # Wait for a key press indefinitely or for a specific amount of time
        cv2.waitKey(0)  # 0 means wait indefinitely until a key is pressed
        
        gray = cv2.cvtColor(src=img, code=cv2.COLOR_BGR2GRAY)

        # Find the checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            objpoints.extend(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), None)
            corners2 = corners2.reshape(-1, 2)
            imgpoints.extend(corners2)
    
    # Check if we have enough points
    if len(objpoints) < 4 or len(imgpoints) < 4:
        print(f"Not enough points were detected. Object points: {len(objpoints)}, Image points: {len(imgpoints)}")
        return

    print(f"Number of valid object points: {len(objpoints)}, image points: {len(imgpoints)}")

    if objpoints and imgpoints:
        # Convert lists to numpy arrays and check dimensions
        objpoints_np = np.array(objpoints, dtype=np.float32)
        objpoints_np = objpoints_np.astype('float32')
        imgpoints_np = np.array(imgpoints, dtype=np.float32)
        imgpoints_np = imgpoints_np.astype('float32')

        # Perform hand-eye calibration
        success, rvecs, tvecs, inliers = cv2.solvePnPRansac(objpoints_np[:600], imgpoints_np[:600], camera_matrix, dist_coefficients)


        if success:
            rotation_matrix, _ = cv2.Rodrigues(rvecs)
            translation_vector = tvecs

            print("Rotation Matrix (R):\n", rotation_matrix)
            print("Translation Vector (T):\n", translation_vector)
        else:
            print("Failed to solve PnP.")
    else:
        print("Not enough valid points were found in the images.")

if __name__ == "__main__": 
    read_data()
    calculate_transformation()
