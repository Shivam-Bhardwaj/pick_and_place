import cv2
import numpy as np
from pypylon import pylon

class Robot_Mapping: 
    # default camera set-up. 
    default_camera_matrix = np.array([[2452.77, 0, 1846.52],
                          [0, 2453.79, 1397.71],
                          [0,  0,  1]], dtype=np.float32)
    default_dist_coefficients = np.array([-0.24818, 0.13173, 0.00068, -0.00053, -0.04507], dtype=np.float32)
    default_grabbing_z_pos = 78.468628
    
    def __init__(self, _camera_matrix = None, _dist_coefficients = None): 
        # Initialize the camera a aruco detectiong
        if _camera_matrix is None: 
            self.camera_matrix = Robot_Mapping.default_camera_matrix
        else: 
            self.camera_matrix = _camera_matrix
        
        if _dist_coefficients is None: 
            self.dist_coefficients = Robot_Mapping.default_dist_coefficients
        else: 
            self.dist_coefficients = _dist_coefficients

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

        # connected to camera
        try: 
            # Connecting to the first available camera
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        except BaseException as e: 
            raise e
    
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

                    # Detect ArUco markers in the image
                    gray = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)
                    corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

                    if ids is not None and len(ids) >= 5:  # Check if at least 5 markers are detected (4 known + 1 new)
                        # Draw the detected markers on the image
                        cv2.aruco.drawDetectedMarkers(undistorted_img, corners, ids)

                        # Dictionary to store the tvecs of detected markers
                        tvecs_dict = {}

                        # Process each detected marker
                        for i in range(len(ids)):
                            marker_id = ids[i][0]
                            retval, rvec, tvec = cv2.solvePnP(self.marker_corners_3d, corners[i][0], self.camera_matrix, self.dist_coefficients)

                            # Convert tvec from meters to inches
                            tvec = tvec.flatten()

                            # Store the tvec for each marker
                            tvecs_dict[marker_id] = tvec

                            # Draw axis using the cv2.drawFrameAxes function
                            cv2.drawFrameAxes(undistorted_img, self.camera_matrix, self.dist_coefficients, rvec, tvec, 0.1)

                        if all(marker_id in tvecs_dict for marker_id in [0, 1, 2, 3]):
                            # Calculate the centroid of the four known markers
                            markers_positions = np.array([tvecs_dict[0], tvecs_dict[1], tvecs_dict[2], tvecs_dict[3]])
                            centroid = np.mean(markers_positions, axis=0)

                            # Assume the new marker ID is the one not in [0, 1, 2, 3]
                            new_marker_id = next(marker_id for marker_id in tvecs_dict if marker_id not in [0, 1, 2, 3])
                            new_marker_position = tvecs_dict[new_marker_id]

                            # Calculate the relative position of the new marker with respect to the centroid
                            relative_position = new_marker_position - centroid
                            
                            # print(f"Update relative position: {relative_position}")
                            relative_position[0] = relative_position[0] * -1000
                            relative_position[1] = relative_position[1] * 1000
                            relative_position[2] = 0
                            # print(f"Relative position of marker {new_marker_id} with respect to the centroid of the four markers: {relative_position}")
                    # # Display the image with detected markers
                    # cv2.namedWindow('Undistorted Image with ArUco Markers', cv2.WINDOW_NORMAL)
                    # cv2.imshow('Undistorted Image with ArUco Markers', undistorted_img)
                grabResult.Release()
            return relative_position
        
        except BaseException as e: 
            raise e
        finally: 
            # always tries to close the camera and all the windows
            self.camera.StopGrabbing()
            cv2.destroyAllWindows()
