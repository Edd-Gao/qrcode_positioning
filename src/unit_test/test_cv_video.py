import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print "failed to open the camera."
    exit(-1)

while True:
    ret, frame = cap.read()
    cv2.imshow("video", frame)
    cv2.waitKey(1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()