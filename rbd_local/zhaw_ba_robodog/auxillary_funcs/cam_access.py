import cv2

cap = cv2.VideoCapture('http://192.168.123.12:8080/?action=stream')
#cap = cv2.VideoCapture(0)

while(True):
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    if cv2.waitKey(10) == 27:  # esc
        cv2.destroyAllWindows()
        cap.release()
        break
