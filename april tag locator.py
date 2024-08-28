import cv2
import numpy as np
from pypylon import pylon

# Define the AprilTag size in meters
TAG_SIZE = 0.039  # 39mm

# Your camera intrinsic parameters (replace with actual values)
camera_matrix = np.array([[2431.93, 0, 1843.19],
                          [0, 2431.40, 1395.97],
                          [0,  0,  1]], dtype=np.float32)

dist_coefficients = np.array([-0.24168, 0.11930, -0.00064, -0.00077, -0.02421], dtype=np.float32)

# Initialize the AprilTag detector using OpenCV
detector = cv2.AprilTagDetector_create()

# Connect to the Basler camera
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

    # Convert to grayscale as AprilTag detection works on grayscale images
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the image
    detector_params = cv2.AprilTagDetector_Params()
    tags = detector.detect(gray, camera_matrix, dist_coeffs, detector_params)

    # List to hold the 3D positions of detected tags
    tag_positions = []

    for tag in tags:
        # Each tag has a dictionary with useful information
        corners = tag.corners.reshape(-1, 2)
        tag_id = tag.id
        rvec = tag.pose_R
        tvec = tag.pose_t

        # The position of the tag in the camera frame
        tag_position = tvec.flatten()
        tag_positions.append(tag_position)

        # Draw the detected tag and its ID on the image
        cv2.polylines(img, [corners.astype(int)], True, (0, 255, 0), 2)
        cv2.putText(img, f"ID: {tag_id}", tuple(corners[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    # Calculate distances between detected tags
    for i in range(len(tag_positions)):
        for j in range(i + 1, len(tag_positions)):
            distance = np.linalg.norm(tag_positions[i] - tag_positions[j])
            print(f"Distance between tag {tags[i].id} and tag {tags[j].id}: {distance:.4f} meters")

    # Show the image with detected tags
    cv2.imshow('AprilTag Detection', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Release the camera
camera.StopGrabbing()
grab_result.Release()
