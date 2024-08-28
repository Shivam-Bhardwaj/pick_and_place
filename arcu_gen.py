import numpy as np
import cv2

# Define the marker size (in pixels) and the border size (optional)
marker_size = 200
border_size = 20

# Define binary patterns for 4 different ArUco markers
marker_patterns = [
    np.array([
        [0, 1, 0, 0, 1, 0],
        [1, 0, 1, 1, 0, 1],
        [0, 1, 0, 1, 0, 1],
        [1, 1, 1, 0, 1, 0],
        [0, 1, 0, 0, 1, 0],
        [1, 0, 1, 1, 0, 1],
    ], dtype=np.uint8),
    np.array([
        [1, 0, 1, 1, 0, 1],
        [0, 1, 0, 0, 1, 0],
        [1, 0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 1],
        [1, 0, 1, 1, 0, 1],
        [0, 1, 0, 0, 1, 0],
    ], dtype=np.uint8),
    np.array([
        [0, 0, 1, 0, 0, 1],
        [1, 1, 0, 1, 1, 0],
        [0, 0, 1, 0, 1, 1],
        [1, 0, 0, 1, 0, 0],
        [0, 1, 1, 0, 1, 1],
        [1, 0, 0, 1, 0, 0],
    ], dtype=np.uint8),
    np.array([
        [1, 1, 0, 1, 0, 0],
        [0, 0, 1, 0, 1, 1],
        [1, 1, 0, 1, 0, 1],
        [0, 1, 1, 0, 1, 1],
        [1, 0, 0, 1, 0, 0],
        [0, 1, 1, 0, 1, 1],
    ], dtype=np.uint8),
]

# Generate and save each marker
for i, marker_pattern in enumerate(marker_patterns):
    # Resize the pattern to the desired marker size
    marker_image = cv2.resize(marker_pattern * 255, (marker_size, marker_size), interpolation=cv2.INTER_NEAREST)

    # Add a white border around the marker
    marker_image = cv2.copyMakeBorder(marker_image, border_size, border_size, border_size, border_size,
                                      cv2.BORDER_CONSTANT, value=255)

    # Save the marker image
    cv2.imwrite(f'manual_aruco_marker_{i}.png', marker_image)

    # Display the marker image (optional)
    cv2.imshow(f'Manual ArUco Marker {i}', marker_image)
    cv2.waitKey(500)  # Display each for 500 ms

cv2.destroyAllWindows()
