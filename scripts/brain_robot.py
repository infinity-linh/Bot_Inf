import time
import cv2
import urllib.request
import numpy as np
from detector import Detection
import threading
import socket
from cacular import *
from model_ANN import ANN
from model_CNN import Net

import torch
from numpy import random

PORT = 8090
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = (SERVER, PORT)
print(SERVER)
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(ADDR)
server.listen()

url1 = 'http://192.168.2.110/cam-lo.jpg'
url2 = 'http://192.168.2.108/cam-lo.jpg'
# url1 = 'http://192.168.1.11/cam-lo.jpg'
# url2 = 'http://192.168.1.9/cam-lo.jpg'
PATH = "D:/User/DLBot/scripts/model/model_auto.pt"
PATH_m = 'D:/User/DLBot/scripts/model/move_model_new_1.pt'

if torch.cuda.is_available():
    device = "cuda:0"
else:
    device = "cpu"

detect = Detection()
model_ctr = ANN(8)
model_ctr.load_state_dict(torch.load(PATH))
net = Net()
net.load_state_dict(torch.load(PATH_m, map_location=torch.device(device)))

boxes_one = []
boxes_two = []
ag = True
image_eye_rootl = []
image_eye_rootr = []
check_cam = False
state_move = False

def change_brightness(img, alpha, beta):
    # cast pixel values to int
    img_new = np.asarray(alpha*img + beta, dtype=int)
    img_new[img_new > 255] = 255
    img_new[img_new < 0] = 0
    return np.array(img_new, dtype=np.uint8)


def eyes(url1, url2):
    global boxes_two
    global boxes_one
    global check_cam
    global image_eye_rootl
    global image_eye_rootr

    # global check_cam_right
    print("Camera ready!")
    while True:
        try:
            img_resp1 = urllib.request.urlopen(url1, timeout=1)
            img_resp2 = urllib.request.urlopen(url2, timeout=1)

            imgnp1 = np.array(bytearray(img_resp1.read()), dtype=np.uint8)
            imgnp2 = np.array(bytearray(img_resp2.read()), dtype=np.uint8)

            image_eye_root1 = cv2.imdecode(imgnp1, 1)
            image_eye_root2 = cv2.imdecode(imgnp2, 1)

            image_eye_root1 = change_brightness(image_eye_root1, 1.0, 35)
            image_eye_root2 = change_brightness(image_eye_root2, 1.0, 35)

            image_eye_root1 = cv2.rotate(
                image_eye_root1, cv2.ROTATE_90_CLOCKWISE)
            image_eye_root2 = cv2.rotate(
                image_eye_root2, cv2.ROTATE_90_CLOCKWISE)

            image_eye_root1 = cv2.resize(image_eye_root1, (400, 400))
            image_eye_root2 = cv2.resize(image_eye_root2, (400, 400))

            image_eye_rootl = image_eye_root1
            image_eye_rootr = image_eye_root2

            boxes1, image_eye_left = detect.detector_image(image_eye_root1)
            boxes2, image_eye_right = detect.detector_image(image_eye_root2)

            if ag == True:
                boxes_one = boxes1
                boxes_two = boxes2

            cv2.imshow("Camera l:", image_eye_left)
            cv2.imshow("Camera r:", image_eye_right)

            check_cam = True

            cv2.waitKey(1)
        except:
            cv2.destroyAllWindows()
            check_cam = False


def control_robot():

    start = [[106, 170, 80, 5],
             [106, 170, 80, 90],
             [106, 48, 100, 90],
             [106, 48, 100, 5],
             [106, 120, 80, 5],
             [10, 150, 80, 5],
             [10, 150, 80, 100],
             [106, 170, 80, 5]]
    global state_move

    idx = 0
    reconect = True
    angles_current = []
    flag_move = True
    state_stop = False
    catch_com = False
    direct = [104,103,102,116]
    idx = 0
    while True:
        angles = []
 
        # catch_com = False

        # state_stop = False
        if reconect == True:
            sv, addr = server.accept()
            reconect = False
            print('Got connection from', addr)
            msg = "255100"+"o"+"\n"
            sv.send(msg.encode())
            time.sleep(1)

        if check_cam == True:
            # print(boxes_one, boxes_two)
            
            msg, flag_move, state_stop, catch_com, angles_current, state_move, idx = state_robot(
                boxes_one, boxes_two, model_ctr, flag_move, state_stop, catch_com, angles_current, state_move, idx)
            # print("msg :", msgs)
            # if len(boxes_one) == 0 and len(boxes_two) == 0:
            #     state_move = True
            # else:
            #     state_move = False
            if state_move != False:
                if len(image_eye_rootl)!=0 and len(image_eye_rootr) != 0:
                    image1 = cv2.resize(image_eye_rootl, [256,256]).transpose([2,0,1])/256
                    image2 = cv2.resize(image_eye_rootr, [256,256]).transpose([2,0,1])/256
                    image1, image2 = torch.tensor([image1],dtype=torch.float32), torch.tensor([image2],dtype=torch.float32)
                    outputs = net(image1, image2)
                    _, predicted = torch.max(outputs, 1)
                    sig = np.array(predicted, dtype=np.uint8)[0]
                    print(direct[sig])
                try:
                    dta, msg = control_v(sv, direct[sig])
                    print("Direct: ",dta)
                    # send_msg = "255400"+str(direct[sig])+"\n"
                    # sv.send(send_msg.encode())
                    # time.sleep(1)
                except:
                    sv.close()
                    reconect = True
            # else :
            #     flag_move = True


            if flag_move == True:
                try:
                    if state_stop == False:
                        idx = 0
                        for i in start[idx]:
                            msg += '0'*(3-len(str(i))) + str(i)
                        msg += '\n'
                    print("Move :", state_move, "msg :", msg)
                    sv.send(msg.encode())
                    time.sleep(3)
                except:
                    sv.close()
                    reconect = True
            elif flag_move == False:
                print("Start catch!")
                try:
                    if state_stop == True:
                        state_stop = False
                        msg = "255100"+"o"+"\n"
                        sv.send(msg.encode())
                        time.sleep(1.5)
                except:
                    sv.close()
                    state_stop = True
                    reconect = True
                if idx == 2:
                    angles = angles_current
                    angles[3] = 90
                    # angles[1] -= 5
                    # angles[2] += 5

                elif idx == 3:
                    angles = angles_current
                    angles[3] = 5
                    # angles[1] -= 5
                    # angles[2] += 5
                else:
                    angles = start[idx]

                print('Angle: ', idx, angles)
                msg = ''
                if angles[0] >= 0 and angles[1] >= 0 and angles[2] >= 0 and angles[3] >= 0:
                    angles = angles
                else:
                    angles = start[idx]
                for i in angles:
                    msg += '0'*(3-len(str(i))) + str(i)
                msg += '\n'
                try:
                    # if state_stop == False:
                    sv.send(msg.encode())
                    time.sleep(0.5)
                    # print('start')
                    if reconect == False and state_stop == False:
                        idx += 1
                except:
                    sv.close()
                    reconect = True
                if idx >= len(start):
                    flag_move = True
                    print(idx, "Stop")
                    idx = 0
                    reconect = True
                    catch_com = True
                    state_move
                    sv.close()
                    time.sleep(2)
        # else:

            # print("Cam left connect: ", check_cam_left)
            # print("Cam connect: ", check_cam)

def moves():
    # PORT = 8090
    # SERVER = socket.gethostbyname(socket.gethostname())
    # ADDR = (SERVER, PORT)
    # print(SERVER)
    # server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # server.bind(ADDR)
    # server.listen()


    direct = [104,103,102,116]
    reconect = True

    while True:
        # if reconect == True:
        #     sv, addr = server.accept()
        #     reconect = False
        #     print('Got connection from', addr)
        #     msg = "255100"+"o"+"\n"
        #     sv.send(msg.encode())
        if state_move != False:
            if len(image_eye_rootl)!=0 and len(image_eye_rootr) != 0:
                image1 = cv2.resize(image_eye_rootl, [255,255]).transpose([2,0,1])/255
                image2 = cv2.resize(image_eye_rootr, [255,255]).transpose([2,0,1])/255
                image1, image2 = torch.tensor([image1],dtype=torch.float32), torch.tensor([image2],dtype=torch.float32)
                outputs = net(image1, image2)
                _, predicted = torch.max(outputs, 1)
                sig = np.array(predicted, dtype=np.uint8)[0]
                print(direct[sig])
            try:
                dta = control_v(sv, direct[sig])
                print("Direct: ",dta)
                # send_msg = "255400"+str(direct[sig])+"\n"
                # sv.send(send_msg.encode())
                time.sleep(1)
            except:
                sv.close()
                reconect = True
if __name__ == '__main__':
    print("Started")

    # t = time.time()
    t1 = threading.Thread(target=eyes, args=(url1, url2,))
    # t2 = threading.Thread(target=moves)
    t4 = threading.Thread(target=control_robot)

 
    t1.start()
    # t2.start()
    # t3.start()
    t4.start()
