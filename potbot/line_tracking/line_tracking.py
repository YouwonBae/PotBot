import cv2 as cv
import numpy as np
import math
import serial
import time

def control_steer(ang):
    sci_angle = ang
    
    return sci_angle

def region_of_interest(img, vertices, color3=(255,255,255), color1=255): #ROI
    mask = np.zeros_like(img)

    if len(img.shape) > 2:  # if color img
        color = color3
    else: #if gray img
        color = color1
    
    cv.fillPoly(mask,vertices, color)
    ROI_image = cv.bitwise_and(img, mask)
    return ROI_image

def van_gradient(left_ang, right_ang, left_x, right_x, left_y, right_y): #get vanishing point
    if(left_ang == right_ang):
        right_ang = 0
        right_y = 720

        arr_A = np.array([[left_ang,-1],[right_ang,-1]])
        arr_B = np.array([((left_ang * left_x) - left_y),((right_ang * right_x) - right_y)])
        van_point = np.linalg.solve(arr_A,arr_B)
        (van_A,van_B) = van_point
        van_A = float(van_A+360-left_x)
        van_B = float(van_B)
    else:
        arr_A = np.array([[left_ang,-1],[right_ang,-1]])
        arr_B = np.array([((left_ang * left_x) - left_y),((right_ang * right_x) - right_y)])
        van_point = np.linalg.solve(arr_A,arr_B)
        (van_A,van_B) = van_point
        van_A = float(van_A)
        van_B = float(van_B)

    if(van_A >= 2000):
        van_A = 2000
    elif(van_A <= -2000):
        van_A = -2000

    if(van_B >= 2000):
        van_B = 2000
    elif(van_B <= -2000):
        van_B = -2000

    if van_B == 0 :
        van_angle = 90
    else:
        van_g = (van_A-360) / van_B
        arctan = np.arctan(van_g)
        van_angle = (180 * (arctan/math.pi))
 
    return van_angle, van_A, van_B

def ang_adj(ang,w,h):
    if(w > h):
        ang = (ang+180)
    else:
        ang = (ang + 90)
    return ang

#----------------------------------------------------------------------------------start line tracking
def line_tracking():
    #variable
    left_x = 0
    right_x = 0
    left_y = 0
    right_y = 0
    half_line = 360
    left_angle = 0
    right_angle = 0
    left_pi_angle = 0
    right_pi_angle = 0
    steer_angle = 0
    van_A = 360
    van_B = 480
    sci_steer = 0
    sci_velo = 0
    
    #serial
    ser = serial.Serial(port="/dev/ttyTHS1", baudrate=115200, timeout=1)
    #my video
    cap = cv.VideoCapture('test.mp4')
    #-----------------------------------------------------------------------------start if
    if cap.isOpened():
        #-------------------------------------------------------------------------start while
        while True:
            ret, img = cap.read() #READ VIDEO
        
            img = cv.flip(img,0) #image adjust
            img = cv.resize(img,(720,480))

            height, width = img.shape[:2]
            vertices = np.array([[(0,0),(0,height/2), (width,height/2),(width,0)]], dtype=np.int32) #ROI select
            ROI_img = region_of_interest(img, vertices) #ROI
            cv.rectangle(img,(0,0),(width,height/2),(0,255,0)) #SHOW ROI

            gray = cv.cvtColor(ROI_img,cv.COLOR_BGR2GRAY)
            ret, thr = cv.threshold(gray,195,255,cv.THRESH_BINARY)

            contour, hierarchy = cv.findContours(thr, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE) #find contour
            #--------------------------------------------------------------------start for
            for i in range(len(contour)):
                cntr = contour[i]
                rect = cv.minAreaRect(cntr)
                (x,y),(w,h),angle = rect

                #########Filtering tiny rectangler#########
                if((w>8 and h>90) or (w>90 and h>8)):
                    box = cv.boxPoints(rect) # contour in box
                    box = np.int0(box)

                    cv.drawContours(img,[box],-1,(0,0,255),3) #draw contour
                    ##############get left,right angle############
                    if(x < half_line):
                        left_angle = int(ang_adj(angle,w,h))
                        left_pi_angle = np.tan((math.pi*left_angle)/180)
                        left_x = int(x)
                        left_y = int(y)
                    else:
                        right_angle = int(ang_adj(angle,w,h))
                        right_pi_angle = np.tan((math.pi*right_angle)/180)
                        right_x = int(x)
                        right_y = int(y)
#---------------------------------------------------------------------------start if
		    if((left_angle > 0) and (right_angle > 0)): #do not permit nagative num
                        #########################Obtuse angle##########################
                        if(right_angle - left_angle <= 0):
                            if(right_angle < 90):
                                right_angle = left_angle
                                right_pi_angle = left_pi_angle
                                right_x = 720-left_x
                                right_y = left_y
                            else:
                                left_angle = right_angle
                                left_pi_angle = right_pi_angle
                                left_x = 720-right_x
                                left_y = right_y
                        ########################One Line detection######################
                        if(left_x == 0 and left_y == 0): #right line only
                            left_angle = right_angle
                            left_pi_angle = right_pi_angle
                            left_x = 720-right_x
                            left_y = right_y
                        elif(right_x == 0 and right_y == 0): #left line only
                            right_angle = left_angle
                            right_pi_angle = left_pi_angle
                            right_x = 720-left_x
                            right_y = left_y
                        ##############steer_angle from vanishing point##############
                        steer_angle, van_A, van_B = van_gradient(left_pi_angle, right_pi_angle, left_x, right_x, left_y, right_y)
                        van_A = int(van_A)
                        van_B = int(van_B)
                        steer_angle = int(steer_angle)
                        #print(steer_angle,van_A,van_B)
                    #----------------------------------------------------------------end negative if
                    ######################show image#####################
                    img = cv.line(img,(left_x,left_y),(van_A,van_B),(0,0,255),3)
                    img = cv.line(img,(right_x,right_y),(van_A,van_B),(0,0,255),3)
                    img = cv.line(img,(360,0),(van_A,van_B),(0,0,255),3)
                    cv.imshow("result",img) #SHOW img
                #---------------------------------------------------------------end box filtering if
                #######################sci work######################
                sci_steer = control_steer(steer_angle)
                sci_velo = 30
                sci_steer = str(sci_steer)
                sci_velo = str(sci_velo)
                Trans = "*"+sci_steer+","+sci_velo+"\n"
                Trans = Trans.encode("utf-8")
                ser.write(Trans)
                print(Trans)

                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
            #--------------------------------------------------------------------end for
        cap.release()
        cv2.destroyAllWindows()
        #------------------------------------------------------------------------end while
    else:
        print("Camera do not working")
    #----------------------------------------------------------------------------end if
#--------------------------------------------------------------------------------end def line tracking
line_tracking()
