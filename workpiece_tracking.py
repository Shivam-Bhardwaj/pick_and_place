import cv2
import numpy as np
from pypylon import pylon

def tracking(self):
    relative_position = None
    try:
        while self.camera.IsGrabbing() and relative_position is None:

            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            if grabResult.GrabSucceeded():
                image = self.converter.Convert(grabResult)
                img = image.GetArray()

                # Undistort the image
                undistorted_img = cv2.undistort(img, self.camera_matrix, self.dist_coefficients, None, self.camera_matrix)

                # Convert the image to grayscale
                gray = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)

                # Use HoughCircles to detect circles in the image
                circles = cv2.HoughCircles(
                    gray, 
                    cv2.HOUGH_GRADIENT, 
                    dp=1.2,  # Inverse ratio of the accumulator resolution to the image resolution
                    minDist=30,  # Minimum distance between the centers of the detected circles
                    param1=100,  # Higher threshold for the Canny edge detector
                    param2=30,  # Accumulator threshold for the circle centers at the detection stage
                    minRadius=10,  # Minimum radius of the circles to detect
                    maxRadius=100  # Maximum radius of the circles to detect
                )

                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    
                    for (x, y, r) in circles:
                        # Draw the circle in the output image
                        cv2.circle(undistorted_img, (x, y), r, (0, 255, 0), 4)
                        # Draw a rectangle at the center of the circle
                        cv2.rectangle(undistorted_img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                        # Convert the center of the circle to a 3D point assuming z = 0
                        circle_center_3d = np.array([x, y, 0], dtype=np.float32)

                        # Transform pixel coordinates to world coordinates using the camera matrix
                        relative_position = cv2.undistortPoints(
                            np.array([[[x, y]]], dtype=np.float32), 
                            self.camera_matrix, 
                            self.dist_coefficients
                        )[0][0]

                        # Assuming the relative position is in the same plane as the ArUco markers
                        # Apply a similar transformation to get the actual coordinates in the robot's frame
                        # Adjust the scale or other transformations as needed for your setup
                        relative_position[0] = relative_position[0] * 1000
                        relative_position[1] = relative_position[1] * 1000
                        relative_position[2] = 0  # Assuming z remains zero

                        break  # Break after the first detected circle

                # Display the image with detected circle
                cv2.namedWindow('Undistorted Image with Circle', cv2.WINDOW_NORMAL)
                cv2.imshow('Undistorted Image with Circle', undistorted_img)
                cv2.waitKey(1)

            grabResult.Release()
        return relative_position

    except BaseException as e:
        raise e
    finally:
        # Always try to close the camera and all the windows
        self.camera.StopGrabbing()
        cv2.destroyAllWindows()
