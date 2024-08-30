import cv2
import os
from pypylon import pylon

# Create the directory to save calibration images if it doesn't exist
output_dir = 'calibration_images'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Connect to the first available camera
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

# Start grabbing continuously (video) with minimal delay
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
converter = pylon.ImageFormatConverter()

# Convert to OpenCV BGR format
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

image_count = 0  # Counter for image filenames

print("Press 'h' to capture an image, or 'q' to quit.")

while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grabResult.GrabSucceeded():
        # Access the image data
        image = converter.Convert(grabResult)
        img = image.GetArray()

        # Display the image
        cv2.namedWindow('Live Feed', cv2.WINDOW_NORMAL)
        cv2.imshow('Live Feed', img)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('h'):
            # Save the current image
            image_count += 1
            image_path = os.path.join(output_dir, f'calibration_{image_count:03d}.jpg')
            cv2.imwrite(image_path, img)
            print(f"Captured and saved {image_path}")

        elif key == ord('q'):
            # Exit the loop
            print("Exiting...")
            break

    grabResult.Release()

# Release the camera and close all windows
camera.StopGrabbing()
cv2.destroyAllWindows()
