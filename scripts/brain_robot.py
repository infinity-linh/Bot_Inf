import time
import cv2
import urllib.request
import numpy as np
from detector import Detection
from threading import Thread
import threading
# from tracker.track import *
import socket
from control_angle import *
from cacular import *
import pandas as pd
import concurrent.futures
from model_ANN import ANN
import torch
from numpy import random

url1 = 'http://192.168.2.110/cam-lo.jpg'
url2 = 'http://192.168.2.108/cam-lo.jpg'
PATH = "D:/User/Bot_C/Res/scripts/model/model.pt"

detect = Detection()
model_ctr = ANN()
model_ctr.load_state_dict(torch.load(PATH))

boxes_one = []
boxes_two = []
ag = True


def change_brightness(img, alpha, beta):
    # cast pixel values to int
    img_new = np.asarray(alpha*img + beta, dtype=int)
    img_new[img_new > 255] = 255
    img_new[img_new < 0] = 0
    return np.array(img_new, dtype=np.uint8)


def cam_right(url):
    global boxes_one
    # img_resp = None
    name = 0
    print("Camera one ready!")
    while True:
        # _, frame = cam.read()
        try:
            img_resp = urllib.request.urlopen(url)
            imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
            if imgnp.any():
                # print(imgnp.shape)
                image_eye_root = cv2.imdecode(imgnp, 1)
                image_eye_left = change_brightness(image_eye_root, 1.0, 35)
                image_eye_left = cv2.rotate(
                    image_eye_left, cv2.ROTATE_90_CLOCKWISE)
                image_eye_left = cv2.resize(image_eye_left, (400, 400))
                boxes, image_eye_left = detect.detector_image(image_eye_left)
                # boxes, img_ori, areas = tracking_sort(image_eye_left)
                if ag == True:
                    # and len(boxes) != 0:
                    boxes_one = boxes
                # print(boxes)
                cv2.imshow("Camera right: ", image_eye_left)
            else:
                cv2.destroyAllWindows()
                # continue
            key = cv2.waitKey(1)
            if key == ord('s'):
                cv2.imwrite("D:/User/data/image/val/imager_{}.png".format(name), image_eye_root)
                name += 1
            if key == ord('x'):
                cv2.destroyAllWindows()
                break
        except:
            pass


def cam_left(url):
    global boxes_two
    print("Camera two ready!")
    name = 0
    while True:
        try:
            img_resp = urllib.request.urlopen(url)
            imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
            if imgnp.any():
                image_eye_root = cv2.imdecode(imgnp, 1)
                # image_eye_right = cv2.flip(image_eye_right, -1)
                image_eye_right = change_brightness(image_eye_root, 1.0, 35)
                image_eye_right = cv2.rotate(
                    image_eye_right, cv2.ROTATE_90_CLOCKWISE)
                image_eye_right = cv2.resize(image_eye_right, (400, 400))
                boxes, image_eye_right = detect.detector_image(image_eye_right)
                if ag == True:
                    # and len(boxes) != 0:
                    boxes_two = boxes
                cv2.imshow("Camera left: ", image_eye_right)
                # continue
                key = cv2.waitKey(1)
                # if key == ord('n'):
                if key == ord('s'):
                    cv2.imwrite("D:/User/data/image/val/imagel_{}.png".format(name), image_eye_root)
                    name += 1
                if key == ord('x'):
                    cv2.destroyAllWindows()
                    break
        except:
            pass
    # cv2.destroyAllWindows()


def auto_robot_v2():
    global ag
    PORT = 8090
    data_create = []
    data_out = []
    data_frame = None
    SERVER = socket.gethostbyname(socket.gethostname())
    ADDR = (SERVER, PORT)
    print(SERVER)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(ADDR)
    server.listen()
    cv2.namedWindow("EYES: ", cv2.WINDOW_AUTOSIZE)
    frame = cv2.imread("D:/User/firmware/Screen/hinh-anh-mat-cuoi2-1.png")
    df = pd.read_csv("D:/User/Bot_C/Res/scripts/control.csv")
    reconect = True
    while True:
        # _, frame = cam.read()
        if reconect == True:
            c, addr = server.accept()
            reconect = False
            print('Got connection from', addr)
        try:
            cv2.imshow("EYES: ", frame)

            key = cv2.waitKey(1)
            if key != -1:
                if len(boxes_one) != 0 and len(boxes_two) != 0:
                    ag = False
                print(key)
                # try:
                if key == ord('m'):
                    ag = True
                    data_create.append(data_frame)
                    data_frame = []
                    # print("Choose data")
                    print("Save data")

                    df2 = pd.DataFrame(data_create, columns=['c11', 'c12', 'c21', 'c22', 'q1', 'q2', 'q3', 'q4'])
                    df2 = pd.concat([df, df2])
                    df2.to_csv(
                        "D:/User/Bot_C/Res/scripts/control.csv", index=False)
                
                if key == ord('n'):
                    c.close()
                    # c, addr = server.accept()
                    reconect = True
                if key == ord('x'):
                    c.close()
                    break
                if key == ord('z'):
                    msg = ''
                    angles = random.randint(180, size=(4))
                    for i in angles:
                        msg += '0'*(3-len(str(i))) + str(i)
                    msg+='\n'
                    print(msg)


                data_out = control_keyboard(c, key)
                if key == ord('v'):
                    msg = ''
                    for i in angles:
                        msg += '0'*(3-len(str(i))) + str(i)
                        msg+='\n'
                    # print(msg)
                if len(boxes_one) != 0 and len(boxes_two) != 0 and data_out != None:
                    print(center_box(boxes_one[0]), center_box(boxes_two[0]), data_out)

                    data_frame = np.hstack(
                        [center_box(boxes_one[0]), center_box(boxes_two[0]), data_out])
                print(data_frame)
                print(data_create)
                # print(ag)
        except:
            c, addr = server.accept()


def control_robot():

    start = [[106, 169, 58, 10],
             [106, 169, 58, 77],
             [108, 48, 100, 77],
             [108, 48, 100, 10],
             [108, 100, 50, 10],
             [23, 150, 39, 10],
             [23, 150, 39, 100],
             [106, 169, 58, 10]]

    PORT = 8090
    SERVER = socket.gethostbyname(socket.gethostname())
    ADDR = (SERVER, PORT)
    print(SERVER)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(ADDR)
    server.listen()
    idx = 0
    reconect = True
    angles_current = []
    flag_move = True

    while True:
        msg = ''
        angles = []

        if reconect == True:
            sv, addr = server.accept()
            reconect = False
            print ('Got connection from', addr )

        if len(boxes_one) != 0 and len(boxes_two) != 0 and flag_move==True:
            boxes = np.array(np.hstack([center_box(boxes_one[0]), center_box(boxes_two[0])]),dtype=np.float32)
            angles = model_ctr(torch.tensor([boxes], dtype=torch.float32))*180
            angles = np.array(angles.detach().numpy()[0], dtype=np.int8)
            angles_current = angles
            flag_move = False
            print("Angles: ", angles)
        elif (len(boxes_one)!= 0 and len(boxes_two)==0):
            # print(boxes_one[0])
            cx, cy = center_box(boxes_one[0])
            if abs(cx - 200) < 100:
                msg = "linh"+"_"+"f"+"\n"
            elif (cx - 200) > 100:
                msg = "linh"+"_"+"t"+"\n"
            elif (cx - 200) < -100:
                msg = "linh"+"_"+"g"+"\n"
        elif (len(boxes_two)!=0 and len(boxes_one)== 0):
            # print(boxes_two[0])
            cx, cy = center_box(boxes_two[0])
            if abs(cx - 200) < 100:
                msg = "linh"+"_"+"t"+"\n"
            elif (cx - 200) > 100:
                msg = "linh"+"_"+"f"+"\n"
            elif (cx - 200) < -100:
                msg = "linh"+"_"+"h"+"\n"
        else:
            msg = "linh"+"_"+"o"+"\n"

        if flag_move == True:
            sv.send(msg.encode())
            time.sleep(0.1)
        elif flag_move == False:
            if idx == 2:
                angles = angles_current 
            else:
                angles = start[idx]
            # print(angles_current)
            print(angles)
            
            msg = ''
            if angles[0] >= 0 and angles[1] >= 0 and angles[2] >= 0 and angles[3] >= 0:
                angles = angles
            else:
                angles = start[idx]
            for i in angles:
                msg += '0'*(3-len(str(i))) + str(i)
            msg+='\n'
            
            # print(idx, len(start)," Box_one: ", boxes_one," Box_two: ", boxes_two, "Angles: ", angles, "Msg: ", msg)
        print(sv.recv(3))
            # sv.send(msg.encode())
        # if sv.recv(3) == b'sta':
        sv.send(msg.encode())
        time.sleep(0.15)
        print('start')
        if reconect == False and sv.recv(3)==b'end':
            idx+=1
        # except:
        #     sv.close()
        #     reconect = True
        if idx >= len(start):
            print(idx, "Stop")
            idx = 0
            flag_move == True
            reconect = True
            sv.close()
            # break
        


if __name__ == '__main__':
    print("Started")

    # t = time.time()
    t1 = threading.Thread(target=cam_left, args=(url1,))
    t2 = threading.Thread(target=cam_right, args=(url2,))
    t3 = threading.Thread(target=auto_robot_v2)
    # t4 = threading.Thread(target=control_robot)

    t1.start()
    t2.start()
    t3.start()
    # t4.start()

    # t1.join()
    # t2.join()
    # t3.join()
    # t4.join()
                                                                                                                                                                                                                                                                        