import cv2
import numpy as np
url = "https://10.27.157.32:8080/video"
cap = cv2.VideoCapture(url)

while(True):
    ret, frame = cap.read()
    frame_resized = cv2.resize(frame,(960,540)) #to resize the frame
#     cv2.imshow("livestream", frame_resized)
    
    gray_im = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)
    clean_im = cv2.GaussianBlur(gray_im,(17,17),0)
#     circles = cv2.HoughCircles(image=clean_im,method=cv2.HOUGH_GRADIENT,dp=0.5,minDist=500,minRadius=250,maxRadius=800)
    circles = cv2.HoughCircles(image=clean_im,method=cv2.HOUGH_GRADIENT,dp=1.2,minDist=500,maxRadius=120)

    if circles is not None:
        circles = np.round(circles[0,:]).astype("int")

    circles_flat = circles.flatten()
    xyrt_list = circles_flat.tolist()
    colourBGR = frame_resized[xyrt_list[1],xyrt_list[0]] #output RGB
    if np.argmax(colourBGR) == 0: #confirm blue
        xyrt_list.append("bowl")
        #colour = "bowl"
    elif np.argmax(colourBGR) == 1: #confirm green
        xyrt_list.append("cup")
        #colour = "cup"
    elif np.argmax(colourBGR) == 2: #red
        xyrt_list.append("plate")
        #colour = "plate"

    print(xyrt_list)
    
    for (x,y,r) in circles:
        cv2.circle(img=frame_resized,center=(x,y),radius=r,color =(255,0,0),thickness=10) #BGR
    
    cv2.imshow('detected_circles',frame_resized)
    
    if cv2.waitKey(1) == ord("q"):
       break
        
cap.release()
cv2.destroyAllWindows()
