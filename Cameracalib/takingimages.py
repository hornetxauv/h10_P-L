import cv2

# Open a connection to the USB camera (0 is usually the default camera)
camera = cv2.VideoCapture(1)
CHECKERBOARD = (6,9)

# Check if the camera opened successfully
if not camera.isOpened():
    print("Error: Could not access the camera.")
    exit()

print("Press 's' to save an image, or 'q' to quit.")
count = 0
while True:
    # Capture frame-by-frame
    ret, frame = camera.read()
    
    # If a frame was not captured, break
    if not ret:
        print("Error: Failed to capture frame.")
        break
    
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD)
    
    #if ret:
     #   cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret)

    # Display the captured frame
    cv2.imshow('Camera', frame)

    # Wait for user input
    key = cv2.waitKey(1) & 0xFF

    # If 's' is pressed, save the image
    if key == ord('s'):
        
        filename = 'image'+str(count) + '.jpg'
        cv2.imwrite(filename, frame)
        print(f"Image saved as {filename}")
        count +=1

    # If 'q' is pressed, quit the loop
    elif key == ord('q'):
        print("Exiting...")
        break

# Release the camera and close the window
camera.release()
cv2.destroyAllWindows()
