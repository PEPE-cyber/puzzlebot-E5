import cv2
import numpy as np

# Initialize webcam
cap = cv2.VideoCapture(0)

# Set the region of interest (ROI) for line detection
roi_top = 300
roi_bottom = 400

# Process each frame from the webcam
while True:
    # Read frame from webcam
    ret, frame = cap.read()

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply adaptive thresholding to create binary image
    _, threshold = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)

    # Extract ROI for line detection
    roi = threshold[roi_top:roi_bottom, :]

    # Find contours in the ROI
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check if any contours are found
    if len(contours) > 0:
        # Find the contour with the largest area
        max_contour = max(contours, key=cv2.contourArea)

        # Get the bounding rectangle of the contour
        x, y, w, h = cv2.boundingRect(max_contour)

        # Calculate the center of the contour
        center_x = x + w // 2

        # Determine the direction based on the center of the contour
        if center_x < frame.shape[1] // 2:
            direction = "Left"
        else:
            direction = "Right"

        # Draw bounding rectangle and center on the frame
        cv2.rectangle(frame, (x, roi_top + y), (x + w, roi_top + y + h), (0, 255, 0), 2)
        cv2.circle(frame, (x + w // 2, roi_top + y + h // 2), 5, (0, 0, 255), -1)

        # Display the direction on the frame
        cv2.putText(frame, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Display the frame
    cv2.imshow("Line Follower", frame)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
