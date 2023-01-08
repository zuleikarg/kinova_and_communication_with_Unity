import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
import numpy as np



class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub_vag = rospy.Publisher('/vagabundeo', Bool, queue_size=10)
        self.vagabundeo = Bool()
        self.vagabundeo.data = False
        self.pub = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=5)
        # Subscribers
        self.sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.callback)


    def callback(self, msg):
        # rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)


    def start(self):
            while True:
                if self.image is not None and self.vagabundeo.data==False:
                    frame = self.image
                    hight,width,_ = frame.shape
                    # Conversión de espacio RGB a HSV
                   
                    HSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                    x,y,w,h=0,0,0,0
                    # # 2- define the range of red
                    # lower=np.array([100, 0, 0])
                    # upper=np.array([255, 255,255])

                    # #check if the HSV of the frame is lower or upper red
                    # Red_mask = cv2.inRange(HSV,lower, upper)
                    # result = cv2.bitwise_and(frame, frame, mask = Red_mask)
                        #Rango y máscara para el color rojo
                    red_lower = np.array([136, 87, 111], np.uint8)
                    red_upper = np.array([180, 255, 255], np.uint8)
                    red_mask = cv2.inRange(HSV, red_lower, red_upper)

                    #Rango y máscara para el color verde
                    green_lower = np.array([25, 52, 72], np.uint8)
                    green_upper = np.array([102, 255, 255], np.uint8)
                    green_mask = cv2.inRange(HSV, green_lower, green_upper)

                    #Rango y máscara para el color azul
                    blue_lower = np.array([94, 80, 2], np.uint8)
                    blue_upper = np.array([120, 255, 255], np.uint8)
                    blue_mask = cv2.inRange(HSV, blue_lower, blue_upper)

                    #Definimos kernel para la operación morfológica
                    kernal = np.ones((5, 5), "uint8")

                    # For red color
                    red_mask = cv2.dilate(red_mask, kernal)
                    res_red = cv2.bitwise_and(frame, frame,
                                            mask=red_mask)

                    # For green color
                    green_mask = cv2.dilate(green_mask, kernal)
                    res_green = cv2.bitwise_and(frame, frame,
                                                mask=green_mask)

                    # For blue color
                    blue_mask = cv2.dilate(blue_mask, kernal)
                    res_blue = cv2.bitwise_and(frame, frame,
                                            mask=blue_mask)

                    # Draw rectangular bounded line on the detected red area
                    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)



                    for pic,contour in enumerate(contours):
                        area = cv2.contourArea(contour)
                        if(area > 5000): #to remove the noise
                            # Constructing the size of boxes to be drawn around the detected red area
                            x_r,y_r,w_r,h_r = cv2.boundingRect(contour)
                            frame = cv2.rectangle(frame, (x_r, y_r), (x_r+w_r, y_r+h_r), (0, 0, 255), 2)

                            cv2.imshow("Tracking Red Color",frame)
                            cv2.imshow("Mask",red_mask)
                            cv2.imshow("And",res_red)

                            # if cv2.waitKey(1) & 0xFF == ord('q'):
                            #     break

                            # # print ("x", x+w)
                            # # print ("y", y+h)
                            # # print("P_hight", hight)
                            # # print("P_width", width/2)
                            # # print("#########################")
                            # giro=(width/2-x-w)/width
                            # avance=(hight-y-h)/hight
                            # # Enviar comandos al robot
                            # print("Avance:",avance)
                            # print("Giro:", giro)
                            # cmd = Twist()
                            # cmd.linear.x = avance*0.5
                            # cmd.angular.z = giro*0.5
                            # self.pub.publish(cmd)

                            # if abs(avance) <= 10**-1 and abs(giro) <= 10**-1:
                            #     print("Vagabundeo")
                            #     self.vagabundeo.data=True
                            #     self.pub_vag.publish(self.vagabundeo)
                            #     cv2.destroyAllWindows()

                     ############ Creating contour to track green color
                    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)        
                    for pic, contour in enumerate(contours):
                        area = cv2.contourArea(contour)
                        if (area > 5000):
                            x_g, y_g, w_g, h_g = cv2.boundingRect(contour)
                            frame = cv2.rectangle(frame, (x_g, y_g),
                                                    (x_g + w_g, y_g + h_g),
                                                    (0, 255, 0), 2)

                            cv2.imshow("Tracking Red Color",frame)
                            cv2.imshow("Mask",green_mask)
                            cv2.imshow("And",res_green)

                    ############# Creating contour to track blue color
                    contours, hierarchy = cv2.findContours(blue_mask,
                                                        cv2.RETR_TREE,
                                                        cv2.CHAIN_APPROX_SIMPLE)
                    for pic, contour in enumerate(contours):
                        area = cv2.contourArea(contour)
                        if (area > 5000):
                            x_b, y_b, w_b, h_b = cv2.boundingRect(contour)
                            frame = cv2.rectangle(frame, (x_b, y_b),
                                                    (x_b + w_b, y_b + h_b),
                                                    (255, 0, 0), 2)
                            cv2.imshow("Tracking Red Color",frame)
                            cv2.imshow("Mask",blue_mask)
                            cv2.imshow("And",res_blue)
                    

                    # TODO: Añadir la función que lea la profundidad de los objetos en cámara

                    # TODO: Movernos hasta un punto cercano (a decidir) a la pelota 





                    # Program Termination
                    cv2.imshow("Multiple Color Detection in Real-TIme", frame)
                    if cv2.waitKey(10) & 0xFF == ord('q'):
                        cap.release()
                        cv2.destroyAllWindows()
                        break                            

    def pase_al_rojo(self):
            while True:
                if self.image is not None and self.vagabundeo.data==False:
                    frame = self.image
                    hight,width,_ = frame.shape
                    # Conversión de espacio RGB a HSV
                
                    HSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                    x,y,w,h=0,0,0,0
                    # # 2- define the range of red
                    # lower=np.array([100, 0, 0])
                    # upper=np.array([255, 255,255])

                    # #check if the HSV of the frame is lower or upper red
                    # Red_mask = cv2.inRange(HSV,lower, upper)
                    # result = cv2.bitwise_and(frame, frame, mask = Red_mask)
                        #Rango y máscara para el color rojo
                    red_lower = np.array([136, 87, 111], np.uint8)
                    red_upper = np.array([180, 255, 255], np.uint8)
                    red_mask = cv2.inRange(HSV, red_lower, red_upper)

                    
                    #Definimos kernel para la operación morfológica
                    kernal = np.ones((5, 5), "uint8")

                    # For red color
                    red_mask = cv2.dilate(red_mask, kernal)
                    res_red = cv2.bitwise_and(frame, frame,
                                            mask=red_mask)

                   
                    # Draw rectangular bounded line on the detected red area
                    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)



                    for pic,contour in enumerate(contours):
                        area = cv2.contourArea(contour)
                        if(area > 5000): #to remove the noise
                            # Constructing the size of boxes to be drawn around the detected red area
                            x_r,y_r,w_r,h_r = cv2.boundingRect(contour)
                            frame = cv2.rectangle(frame, (x_r, y_r), (x_r+w_r, y_r+h_r), (0, 0, 255), 2)

                            cv2.imshow("Tracking Red Color",frame)
                            cv2.imshow("Mask",red_mask)
                            cv2.imshow("And",res_red)

                            # if cv2.waitKey(1) & 0xFF == ord('q'):
                            #     break

                            # # print ("x", x+w)
                            # # print ("y", y+h)
                            # # print("P_hight", hight)
                            # # print("P_width", width/2)
                            # # print("#########################")
                            # giro=(width/2-x-w)/width
                            # avance=(hight-y-h)/hight
                            # # Enviar comandos al robot
                            # print("Avance:",avance)
                            # print("Giro:", giro)
                            # cmd = Twist()
                            # cmd.linear.x = avance*0.5
                            # cmd.angular.z = giro*0.5
                            # self.pub.publish(cmd)

                            # if abs(avance) <= 10**-1 and abs(giro) <= 10**-1:
                            #     print("Vagabundeo")
                            #     self.vagabundeo.data=True
                            #     self.pub_vag.publish(self.vagabundeo)
                            #     cv2.destroyAllWindows()

                   




if __name__ == '__main__':
    rospy.init_node("tracking", anonymous=True)
    my_node = Nodo()
    my_node.start()
    #my_node.pase_al_rojo()
