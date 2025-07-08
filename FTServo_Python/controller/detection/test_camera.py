import cv2

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    assert frame is not None
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break