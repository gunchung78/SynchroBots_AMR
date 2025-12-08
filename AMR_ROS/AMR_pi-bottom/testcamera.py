import cv2

cap = cv2.VideoCapture(0)  # /dev/video0

ret, frame = cap.read()
if ret:
    h, w = frame.shape[:2]
    print("width =", w)
    print("height =", h)
else:
    print("fail")

cap.release()