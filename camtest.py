import cv2

# Quick test to make sure OpenCV can access camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Camera not accessible")
    exit()

ret, frame = cap.read()
if ret:
    cv2.imwrite('camera_test.jpg', frame)
    print("Success! Check camera_test.jpg")
else:
    print("Failed to capture frame")

cap.release()
