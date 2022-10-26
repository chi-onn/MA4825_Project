import cv2
import numpy as np

url = "https://10.27.157.32:8080/video"
cap = cv2.VideoCapture(url)

ret, frame = cap.read()

if ret:
    frame_resized = cv2.resize(frame,(960,540)) #to resize the frame
    cv2.imwrite("data.jpg",frame_resized)

image = cv2.imread("data.jpg")

cv2.line(img=image,pt1=(0,0),pt2=(200,0),color =(255,0,0),thickness=10) #blue: x-axis
cv2.line(img=image,pt1=(0,0),pt2=(0,200),color =(0,0,255),thickness=10) #red: x-axis

while True:
    cv2.imshow("test",image)
    
    if cv2.waitKey(1) == ord("q"):
       break