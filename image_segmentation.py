import cv2
import numpy as np

# Load the image
image = cv2.imread('i10.png')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply GaussianBlur to reduce noise and make the edges smoother
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# Use Canny edge detection
edges = cv2.Canny(blurred, 20, 200)

# Find contours in the edged image
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# List to store areas and centers
shapes_info = []

# Loop over each contour
for contour in contours:
    # Get the moments to calculate the center of the contour
    M = cv2.moments(contour)
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # Calculate the area of the contour
        area = cv2.contourArea(contour)

        # Store the area and center
        shapes_info.append((area, cx, cy, contour))

# Sort the shapes by area in descending order (largest first)
shapes_info.sort(reverse=True, key=lambda x: x[0])

# Loop through sorted shapes and annotate them with rank
for rank, (area, cx, cy, contour) in enumerate(shapes_info, start=1):
    # Fit a minimum enclosing circle to detect if it's a circle or square
    (_, radius) = cv2.minEnclosingCircle(contour)
    circularity = 4 * np.pi * (area / (cv2.arcLength(contour, True) ** 2))

    # Determine if the shape is a circle or square based on circularity
    if 0.85 < circularity < 1.15:
        shape = "Circle"
    else:
        shape = "Square"

    # Draw the center point and shape on the image
    cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
    cv2.putText(image, f"{shape} Rank {rank}", (cx - 40, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Print the center and rank information
    print(f"Shape: {shape}, Center: ({cx}, {cy}), Area: {area}, Rank: {rank}")

# Save and display the final image with centers and ranks marked
final_image_path = 'your_image_with_centers_and_ranks.png'
cv2.imwrite(final_image_path, image)
cv2.imshow('Detected Centers and Ranks', image)

# Wait for a key press and close all windows
cv2.waitKey(0)
cv2.destroyAllWindows()
