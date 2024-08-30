import cv2
import numpy as np
from pypylon import pylon

class Robot_Mapping: 
    # Default camera set-up
    default_camera_matrix = np.array([[2452.77, 0, 1846.52],
                                      [0, 2453.79, 1397.71],
                                      [0,  0,  1]], dtype=np.float32)
    default_dist_coefficients = np.array([-0.24818, 0.13173, 0.00068, -0.00053, -0.04507], dtype=np.float32)
    
    def __init__(self, _camera_matrix=None, _dist_coefficients=None): 
        # Initialize the camera and ArUco detection
        self.camera_matrix = _camera_matrix if _camera_matrix is not None else Robot_Mapping.default_camera_matrix
        self.dist_coefficients = _dist_coefficients if _dist_coefficients is not None else Robot_Mapping.default_dist_coefficients

        # Converter to OpenCV format
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        # Define the ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        # Define the 3D points of the ArUco marker corners in the marker's coordinate system
        self.marker_size = 0.05  # size of the marker (in meters)
        self.marker_corners_3d = np.array([
            [-self.marker_size / 2, -self.marker_size / 2, 0],
            [ self.marker_size / 2, -self.marker_size / 2, 0],
            [ self.marker_size / 2,  self.marker_size / 2, 0],
            [-self.marker_size / 2,  self.marker_size / 2, 0]
        ], dtype=np.float32)

        # Connect to the camera
        try: 
            # Connecting to the first available camera
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        except BaseException as e: 
            raise e

    def on_trackbar(self, val):
        pass  # Dummy function for trackbar

    def find_circle_relative_to_markers(self):
        # Create window for sliders
        cv2.namedWindow('Parameters', cv2.WINDOW_NORMAL)
        
        # Create sliders
        cv2.createTrackbar('minRadius', 'Parameters', 1, 200, self.on_trackbar)
        cv2.createTrackbar('maxRadius', 'Parameters', 200, 5000, self.on_trackbar)
        cv2.createTrackbar('param1', 'Parameters', 1, 255, self.on_trackbar)
        cv2.createTrackbar('param2', 'Parameters', 1, 255, self.on_trackbar)

        try:
            while self.camera.IsGrabbing():
                grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

                if grabResult.GrabSucceeded():
                    image = self.converter.Convert(grabResult)
                    img = image.GetArray()

                    # Undistort the image
                    undistorted_img = cv2.undistort(img, self.camera_matrix, self.dist_coefficients, None, self.camera_matrix)

                    # Convert the image to grayscale
                    gray = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)

                    # Apply GaussianBlur to reduce noise and make the edges smoother
                    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

                    # Get current values from sliders
                    minRadius = cv2.getTrackbarPos('minRadius', 'Parameters')
                    maxRadius = cv2.getTrackbarPos('maxRadius', 'Parameters')
                    param1 = cv2.getTrackbarPos('param1', 'Parameters')
                    param2 = cv2.getTrackbarPos('param2', 'Parameters')

                    # Detect ArUco markers in the image
                    corners, ids, rejected = cv2.aruco.detectMarkers(blurred, self.aruco_dict, parameters=self.parameters)

                    if ids is not None and len(ids) >= 4:  # Check if at least 4 markers are detected
                        tvecs_dict = {}

                        # Estimate pose for each detected ArUco marker
                        for i in range(len(ids)):
                            marker_id = ids[i][0]
                            retval, rvec, tvec = cv2.solvePnP(self.marker_corners_3d, corners[i][0], self.camera_matrix, self.dist_coefficients)

                            # Convert tvec from meters to millimeters and store it
                            tvec_mm = tvec.flatten() * 1000
                            tvecs_dict[marker_id] = tvec_mm

                            # Draw the detected markers and their axes on the image
                            cv2.drawFrameAxes(undistorted_img, self.camera_matrix, self.dist_coefficients, rvec, tvec, 0.1)
                            cv2.aruco.drawDetectedMarkers(undistorted_img, corners, ids)

                        if all(marker_id in tvecs_dict for marker_id in [0, 1, 2, 3]):
                            # Calculate the centroid of the four known markers
                            markers_positions = np.array([tvecs_dict[0], tvecs_dict[1], tvecs_dict[2], tvecs_dict[3]])
                            centroid = np.mean(markers_positions, axis=0)

                            # Detect the circular object
                            circles = cv2.HoughCircles(
                                blurred, 
                                cv2.HOUGH_GRADIENT, 
                                dp=1.0,  # Inverse ratio of the accumulator resolution to the image resolution
                                minDist=30,  # Minimum distance between the centers of the detected circles
                                param1=param1,  # Higher threshold for the Canny edge detector
                                param2=param2,  # Accumulator threshold for the circle centers at the detection stage
                                minRadius=minRadius,  # Minimum radius of the circles to detect
                                maxRadius=maxRadius  # Maximum radius of the circles to detect
                            )

                            if circles is not None:
                                circles = np.round(circles[0, :]).astype("int")
                                
                                for (x, y, r) in circles:
                                    # Draw the circle in the output image
                                    cv2.circle(undistorted_img, (x, y), r, (0, 255, 0), 4)
                                    # Draw a rectangle at the center of the circle
                                    cv2.rectangle(undistorted_img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                                    # Convert the center of the circle to a 3D point assuming z = 0
                                    circle_center_2d = np.array([[x, y]], dtype=np.float32)
                                    circle_center_world = cv2.undistortPoints(circle_center_2d, self.camera_matrix, self.dist_coefficients)[0][0] * 1000  # Convert to mm

                                    # Calculate the relative position to the centroid of the ArUco markers
                                    relative_position = circle_center_world - centroid[:2]
                                    print(f"Circle center relative to centroid in mm: {relative_position}")

                                    break  # Break after the first detected circle

                    # Display the image with detected markers and circle
                    cv2.namedWindow('Detected Markers and Circle', cv2.WINDOW_NORMAL)
                    cv2.imshow('Detected Markers and Circle', undistorted_img)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                grabResult.Release()

        except BaseException as e:
            raise e
        finally:
            # Always try to close the camera and all the windows
            self.camera.StopGrabbing()
            cv2.destroyAllWindows()

# Example of how to use the class
if __name__ == "__main__":
    robot_mapping = Robot_Mapping()
    robot_mapping.find_circle_relative_to_markers()
