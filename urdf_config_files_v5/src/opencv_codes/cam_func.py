import cv2
import numpy as np

def output_coordinate():
    url = "https://192.168.0.109:8080/video"
    cap = cv2.VideoCapture(url)

    #### TO CAPTURE IMAGE ####
    ret, frame = cap.read()

    if ret:
        frame_resized = cv2.resize(frame,(960,540)) #to resize the frame
        cv2.imwrite("data.jpg",frame_resized)

    image = cv2.imread("data.jpg")

    #### TO PROCESS IMAGE ####
    gray_im = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    clean_im = cv2.GaussianBlur(gray_im,(17,17),0)

    #### TO DETECT CIRCLE ####
    circles = cv2.HoughCircles(image=clean_im,method=cv2.HOUGH_GRADIENT,dp=1.2,minDist=500,maxRadius=120) #PARAMETERS TO BE TUNED!!

    if circles is not None:
        circles = np.round(circles[0,:]).astype("int")

    # print(circles) ###for debug
    #### TO DETECT TYPE (PLATE/CUP/BOWL) ####
    circles_flat = circles.flatten()
    xyrt_list = circles_flat.tolist()

    # print("before:",xyrt_list) ###for debug
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

#     print(xyrt_list)

    #### TO CONVERT TO COORDINATES ####
    """
    at the height of 27cm (35cm from the floor) from white container:
    1px -> 0.45mm hence, scaling factor (k) = 0.45

    x,y,z coordinates are expressed in mm
    """
    k = 0.45

    d_robot_tray_x = 0
    d_robot_tray_y = 200
    water_level = 40

    ## equations ##
    x_center = - d_robot_tray_x + ( k * xyrt_list[1] )
    y_center = d_robot_tray_y + ( k * xyrt_list[0] )

    if xyrt_list[3] == "bowl":
        x_co = x_center 
        y_co = y_center - 40 #diameter of bowl = 82mm
        z_co = water_level + 100
    if xyrt_list[3] == "plate":
        x_co = x_center
        y_co = y_center
        z_co = water_level
    if xyrt_list[3] == "cup":
        x_co = x_center 
        y_co = y_center - 21 #diameter of cup = 43mm
        z_co = water_level + 30
    
    coordinates = x_co, y_co, z_co, xyrt_list[3]
    
    #### TO DISPLAY ####
#     for (x,y,r) in circles:
#         cv2.circle(img=image,center=(x,y),radius=r,color =(255,0,0),thickness=10) #BGR

#     while True:
#         cv2.imshow("test",image)
    
#         if cv2.waitKey(1) == ord("q"):
#            break
    return coordinates

# coordinates = output_coordinate()
# print(coordinates)

