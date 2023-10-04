import time
import cv2, os
import urllib.request
import numpy as np
from detector import Detection
from threading import Thread
import threading
# from tracker.track import *
import socket
# from control_angle import *
from cacular import *
import pandas as pd
from model_ANN import ANN
import torch
from numpy import random

url1 = 'http://192.168.2.110/cam-lo.jpg'
url2 = 'http://192.168.2.108/cam-lo.jpg'
# url1 = 'http://192.168.1.11/cam-lo.jpg'
# url2 = 'http://192.168.1.9/cam-lo.jpg'



PATH = "D:/User/Bot_C/Res/scripts/model/model_auto.pt"

detect = Detection()
model_ctr = ANN(8)
model_ctr.load_state_dict(torch.load(PATH))

boxes_one = []
boxes_two = []
ag = True
image_eye_rootl = None
image_eye_rootr = None
check_cam = False


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
        if check_cam == False:
            cv2.namedWindow('Camera l:', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Camera r:', cv2.WINDOW_NORMAL)

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
            check_cam = True

            cv2.imshow("Camera l:", image_eye_left)
            cv2.imshow("Camera r:", image_eye_right)


            cv2.waitKey(1)
        except:
            # cv2.destroyAllWindows()
            cv2.destroyWindow('Camera l:')
            cv2.destroyWindow('Camera r:')

            check_cam = False


def auto_robot_v2():
    global ag
    PORT = 8090
    data_create = []
    data_frame = None
    auto_run = False
    SERVER = socket.gethostbyname(socket.gethostname())
    ADDR = (SERVER, PORT)
    print(SERVER)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(ADDR)
    server.listen()
    path_csv = "D:/User/DLBot/scripts/data/angles.csv"

    df = pd.read_csv(path_csv)
    reconect = True
    angles_save = None

    while True:
        # _, frame = cam.read()
        # print(boxes_one, boxes_two)
        if reconect == True:
            c, addr = server.accept()
            reconect = False
            print('Got connection from', addr)
        if check_cam == True:
            # try:

                # if len(boxes_one) != 0 and len(boxes_two) != 0:
                #     ag = False

                # if key == ord('z'):
                auto_run = True
                print("Auto run start!", auto_run)

                if auto_run == True:
                    msg = ''
                    angles_base = random.randint(80, 110)
                    angles_show = random.randint(35, 60)
                    angles_elbo = random.randint(50, 100)
                    angles_grip = random.randint(10, 15)

                    angles = [angles_base, angles_show, angles_elbo, angles_grip]
                    angles_save = angles
                    for i in angles:
                        msg += '0'*(3-len(str(i))) + str(i)
                    msg += '\n'
                    try :
                        print(msg)
                        c.send(msg.encode())
                        time.sleep(3)
                    except:
                        c.close()
                        reconect = True

                if len(boxes_one) != 0 and len(boxes_two) != 0 and angles_save != None and auto_run == True:
                    # set_angles = angles_save
                    print(center_box(boxes_one[0]), center_box(
                        boxes_two[0]), angles_save)
                    data_frame = np.hstack(
                        [boxes_one[0], boxes_two[0], angles_save])
                    if square_box(boxes_one[0]) < 50000 and square_box(boxes_two[0]) < 50000:
                        # ag = True
                        data_create.append(data_frame)
                        data_frame = []
                        # print("Choose data")
                        print("Save data")

                        df2 = pd.DataFrame(data_create, columns=['x1', 'x2', 'x3', 'x4',
                                                                    'y1', 'y2', 'y3', 'y4',
                                                                    'q1', 'q2', 'q3', 'q4'])

                        # df2 = pd.DataFrame(data_create, columns=['c11', 'c12', 'c21', 'c22', 'q1', 'q2', 'q3', 'q4'])
                        df2 = pd.concat([df, df2])
                        df2.to_csv(
                            path_csv, index=False)

                    print(data_create)
            # except:
            #     c.close()
            #     reconect = False


def create_data():
    # my_room = np.ones((20,8,3))
    # print(my_room[1,1])
    PORT = 8090
    SERVER = socket.gethostbyname(socket.gethostname())
    ADDR = (SERVER, PORT)
    print(SERVER)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(ADDR)
    server.listen()
    reconect = True
    path_csv = "D:/User/DLBot/scripts/data/locals.csv"

    path_save_l = "D:/User/data_map/left/"
    path_save_r = "D:/User/data_map/right/"
    idx_name = len(os.listdir(path_save_l))
    print(idx_name)
    # cv2.namedWindow("Eye:", cv2.WINDOW_NORMAL)


    while True:
        if reconect == True:
            frame = cv2.imread("D:/User/firmware/Screen/hinh-anh-mat-cuoi2-1.png")
            cv2.namedWindow("Eye:", cv2.WINDOW_NORMAL)
            sv, addr = server.accept()
            reconect = False
            print('Got connection from', addr)

        if check_cam == True:
            cv2.imshow("Eye:", frame)
            key = cv2.waitKey(1)

            try:
                if key != -1:
                    data_frame = []
                    df = pd.read_csv(path_csv)
                    name_l = "imagel_{}.png".format(idx_name)
                    name_r = "imager_{}.png".format(idx_name)
                    cv2.imwrite(path_save_l+name_l, image_eye_rootl)
                    cv2.imwrite(path_save_r+name_r, image_eye_rootr)
                    data_out, _ = control_v(sv, key)
                    # data_pre = data_out
                    # if data_out != 't':
                    data_frame.append(name_l)
                    data_frame.append(name_r)
                    data_frame.append(data_out)
                    df2 = pd.DataFrame([data_frame], columns=[
                                    'imagel', 'imager', 'action'])
                    df2 = pd.concat([df, df2])
                    df2.to_csv(
                        path_csv, index=False)
                    print(data_frame)
                    idx_name += 1
                    # time.sleep(2)
            except:
                sv.close()
                reconect = True
        else:
            sv.close()
            cv2.destroyWindow("Eye:")
            print("Status camera:", check_cam)
            reconect = True

def colection_data():
    path_save_l = "D:/User/data_object/images/"
    idx_name = len(os.listdir(path_save_l))
    while True:
        try:
            name_l = "imagel_{}.png".format(idx_name)
            print(name_l)
            cv2.imwrite(path_save_l+name_l, image_eye_rootl)
            idx_name+=1
            time.sleep(3)
        except:
            pass

if __name__ == '__main__':
    print("Started")

    # t = time.time()
    t1 = threading.Thread(target=eyes, args=(url1, url2,))
    # t2 = threading.Thread(target=create_data)
    t3 = threading.Thread(target=auto_robot_v2)
    # t3 = threading.Thread(target=colection_data)


    t1.start()
    # t2.start()
    t3.start()

