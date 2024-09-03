import cv2
import cv2.aruco as aruco
import numpy as np


# Initialize the webcam
cap = cv2.VideoCapture(4)  # Replace 4 with the correct index if needed

# Load the predefined dictionary for ArUco markers
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Real-world coordinates of ArUco markers (in millimeters)
aruco_positions = {
    0: (0.0, 0.0),
    1: (-685.8, 0.0),       # Converted to millimeters
    2: (-685.8, 330.2),      # Converted to millimeters
    3: (0.0, 330.2)          # Converted to millimeters
}

# Prepare to store the pixel coordinates of the ArUco markers
pixel_positions = {}

if not cap.isOpened():
    print("Error: Could not open video stream from webcam.")
    exit()

while True:
    try: 
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Failed to capture image.")
            break

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers in the frame
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            # For each detected ArUco marker, store its pixel coordinates
            for i in range(len(ids)):
                marker_id = ids[i][0]
                corner = corners[i][0]
                # Calculate the center of the ArUco marker in pixels
                cX = int(np.mean(corner[:, 0]))
                cY = int(np.mean(corner[:, 1]))
                pixel_positions[marker_id] = (cX, cY)
                
                # Mark the center of each ArUco marker with a circle
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"ID: {marker_id}", (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            if len(pixel_positions) == 4:  # Ensure all markers are detected
                # Now that you have both pixel and real-world positions, calculate the transformation
                
                # Extract points for cv2.getPerspectiveTransform
                src_points = np.array([pixel_positions[0], pixel_positions[1], pixel_positions[2], pixel_positions[3]], dtype="float32")
                dst_points = np.array([aruco_positions[0], aruco_positions[1], aruco_positions[2], aruco_positions[3]], dtype="float32")

                # Calculate the perspective transform matrix
                matrix = cv2.getPerspectiveTransform(src_points, dst_points)

                # Create a mask to cover the ArUco markers
                mask = np.ones(gray.shape, dtype=np.uint8) * 255  # Start with a white mask

                for corner in corners:
                    int_corners = np.int32(corner)  # Convert corners to integer using np.int32
                    cv2.fillPoly(mask, [int_corners], 0)  # Fill the polygon with black on the mask

                # Apply the mask to the original grayscale image
                masked_gray = cv2.bitwise_and(gray, gray, mask=mask)
                
                # Apply a Gaussian blur to reduce noise and improve detection accuracy
                blurred = cv2.GaussianBlur(masked_gray, (5, 5), 0)
                
                # Threshold the image to get a binary image
                _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)
                
                # Find contours in the thresholded image
                contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                circles_info = []
                
                for contour in contours:
                    # Approximate the contour to find shapes
                    approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)

                    # Detect circles
                    if len(approx) > 4:
                        area = cv2.contourArea(contour)
                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        circularity = area / (np.pi * (radius ** 2))
                        if 0.7 <= circularity <= 1.3:  # Circularity check
                            # Convert the pixel center to real-world coordinates
                            circle_center_img = np.array([[x, y]], dtype="float32")
                            circle_center_world = cv2.perspectiveTransform(np.array([circle_center_img]), matrix)
                            circle_center_world = circle_center_world[0][0]

                            # Store the circle's information
                            circles_info.append({
                                'area': area,
                                'radius': radius,
                                'pixel_center': (int(x), int(y)),
                                'world_center': (circle_center_world[0], circle_center_world[1])  # In millimeters
                            })
                
                # Sort the circles by area from smallest to largest
                circles_info.sort(key=lambda c: c['area'])


                # Display the circles with their area ranks and real-world coordinates
                for i, circle in enumerate(circles_info):
                    # Draw the circle
                    cv2.circle(frame, circle['pixel_center'], int(circle['radius']), (255, 0, 0), 2)
                    # Draw the center of the circle
                    cv2.circle(frame, circle['pixel_center'], 5, (0, 255, 0), -1)
                    # Display the area ranking and real-world coordinates (without units)
                    cv2.putText(frame, f"Rank: {i+1}",
                                (circle['pixel_center'][0] + 10, circle['pixel_center'][1] - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, f"({circle['world_center'][0]:.2f}, {circle['world_center'][1]:.2f})",
                                (circle['pixel_center'][0] + 10, circle['pixel_center'][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Display the frame with detected circles, area ranks, and coordinates
        cv2.imshow("Circle Detection with Area Ranking and Real-World Coordinates", frame)
        
        # Add a small delay to reduce CPU usage and allow for quitting
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    except BaseException as e: 
        # Release the capture and close the windows
        cap.release()
        cv2.destroyAllWindows()
        raise e

# Release the capture and close the windows
cap.release()
cv2.destroyAllWindows()
