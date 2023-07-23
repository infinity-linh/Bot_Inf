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
PATH = "D:/User/Bot_C/Res/scripts/model/model_auto.pt"

detect = Detection()
model_ctr = ANN(8)
model_ctr.load_state_dict(torch.load(PATH))

boxes_one = []
boxes_two = []
ag = True
check_cam_right = False
check_cam_left = False


def change_brightness(img, alpha, beta):
    # cast pixel values to int
    img_new = np.asarray(alpha*img + beta, dtype=int)
    img_new[img_new > 255] = 255
    img_new[img_new < 0] = 0
    return np.array(img_new, dtype=np.uint8)


def cam_right(url):
    global boxes_one
    global check_cam_right

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
                image_eye_root = change_brightness(image_eye_root, 1.0, 35)
                image_eye_root = cv2.rotate(
                    image_eye_root, cv2.ROTATE_90_CLOCKWISE)
                image_eye_root = cv2.resize(image_eye_root, (400, 400))
                boxes, image_eye_left = detect.detector_image(image_eye_root)
                # boxes, img_ori, areas = tracking_sort(image_eye_left)
                if ag == True:
                    # and len(boxes) != 0:
                    boxes_one = boxes
                # print(boxes)
                cv2.imshow("Camera right: ", image_eye_left)
                check_cam_right = True
            else:
                check_cam_right = False
                cv2.destroyAllWindows()
                # continue
            key = cv2.waitKey(1)
            if key == ord('s'):
                cv2.imwrite(
                    "D:/User/data/image/val/imager_{}.png".format(name), image_eye_root)
                name += 1
            if key == ord('x'):
                cv2.destroyAllWindows()
                break
        except:
            pass


def cam_left(url):
    global boxes_two
    global check_cam_left

    print("Camera two ready!")
    name = 0
    while True:
        try:
            img_resp = urllib.request.urlopen(url)
            imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
            if imgnp.any():
                image_eye_root = cv2.imdecode(imgnp, 1)
                # image_eye_right = cv2.flip(image_eye_right, -1)
                image_eye_root = change_brightness(image_eye_root, 1.0, 35)
                image_eye_root = cv2.rotate(
                    image_eye_root, cv2.ROTATE_90_CLOCKWISE)
                image_eye_root = cv2.resize(image_eye_root, (400, 400))
                boxes, image_eye_right = detect.detector_image(image_eye_root)
                if ag == True:
                    # and len(boxes) != 0:
                    boxes_two = boxes
                cv2.imshow("Camera left: ", image_eye_right)
                # continue
                check_cam_left = True

            else:
                check_cam_left = False
                cv2.destroyAllWindows()
            key = cv2.waitKey(1)
            # if key == ord('n'):
            if key == ord('s'):
                cv2.imwrite(
                    "D:/User/data/image/val/imagel_{}.png".format(name), image_eye_root)
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
    auto_run = False
    set_angles = []
    SERVER = socket.gethostbyname(socket.gethostname())
    ADDR = (SERVER, PORT)
    print(SERVER)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(ADDR)
    server.listen()
    cv2.namedWindow("EYES: ", cv2.WINDOW_AUTOSIZE)
    frame = cv2.imread("D:/User/firmware/Screen/hinh-anh-mat-cuoi2-1.png")
    # path_csv = "D:/User/Bot_C/Res/scripts/control.csv"
    path_csv = "D:/User/Bot_C/Res/scripts/angles.csv"

    df = pd.read_csv(path_csv)
    reconect = True
    angles_save = None

    while True:
        # _, frame = cam.read()
        print(boxes_one, boxes_two)
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
                # print(key)
                # try:
                if key == ord('m'):
                    ag = True
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

                elif key == ord('n'):
                    c.close()
                    auto_run = False
                    # c, addr = server.accept()
                    reconect = True
                elif key == ord('x'):
                    c.close()
                    break

                elif key == ord('v'):
                    msg = ''
                    for i in set_angles:
                        msg += '0'*(3-len(str(i))) + str(i)
                    msg += '\n'
                    c.send(msg.encode())
                    # time.sleep(0.15)
                else:
                    data_out = control_keyboard(c, key)
                    angles_save = data_out
                    # print(msg)
                if key == ord('z'):
                    auto_run = True
                    print("Auto run start!", auto_run)

            if auto_run == True:
                msg = ''
                angles_base = random.randint(80, 120)
                angles_grip = random.randint(20, 25)
                angles_show = random.randint(50, 80)
                angles_elbo = random.randint(50, 120)
                angles = [angles_base, angles_show, angles_elbo, angles_grip]
                angles_save = angles
                for i in angles:
                    msg += '0'*(3-len(str(i))) + str(i)
                msg += '\n'
                print(msg)
                c.send(msg.encode())
                time.sleep(3)

            if len(boxes_one) != 0 and len(boxes_two) != 0 and angles_save != None and auto_run == True:
                set_angles = angles_save
                print(center_box(boxes_one[0]), center_box(
                    boxes_two[0]), angles_save)
                # print(angles_save)
                # data_frame = np.hstack(
                #     [center_box(boxes_one[0]), center_box(boxes_two[0]), angles_save])
                data_frame = np.hstack(
                    [boxes_one[0], boxes_two[0], angles_save])
                if square_box(boxes_one[0]) < 90000 and square_box(boxes_two[0]) < 90000:
                    ag = True
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
            # print(data_frame)
                print(data_create)
                # print(ag)
        except:
            c.close()
            # c, addr = server.accept()
            reconect = True
            # c, addr = server.accept()


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
    state_stop = False
    count = 0
    ctr = 0
    while True:
        msg = ''
        angles = []
        area_one, area_two = 0, 0
        # state_stop = False
        if check_cam_right == True and check_cam_left == True:

            if reconect == True:
                sv, addr = server.accept()
                reconect = False
                print('Got connection from', addr)
            print(len(boxes_one), len(boxes_two))

            if len(boxes_one) != 0 and len(boxes_two) != 0:
                # and flag_move==True:
                # boxes = np.array([np.hstack([center_box(boxes_one[0]), center_box(boxes_two[0])])/400],dtype=np.float32)
                boxes = np.array(
                    np.hstack([boxes_one[0], boxes_two[0]])/400, dtype=np.float32)

                angles = model_ctr(torch.tensor(
                    boxes, dtype=torch.float32))*180
                angles = np.array(angles.detach().numpy(), dtype=np.int8)
                angles_current = angles
                flag_move = False
                state_stop = True
                # print("Angles: ", angles_current)
            # else:
            #     flag_move = True
            elif (len(boxes_one) != 0 and len(boxes_two) == 0):
                # flag_move = True
                # print(boxes_one[0])
                area_one = square_box(boxes_one[0])
                cx, cy = center_box(boxes_one[0])
                if abs(cx - 200) < 100 and 0 < area_one < 7000:
                    msg = "linh"+"_"+"h"+"\n"
                elif (cx - 200) > 100:
                    msg = "linh"+"_"+"t"+"\n"
                elif (cx - 200) < -100:
                    msg = "linh"+"_"+"g"+"\n"
            elif (len(boxes_two) != 0 and len(boxes_one) == 0):
                # flag_move = True
                # print(boxes_two[0])
                area_two = square_box(boxes_two[0])
                cx, cy = center_box(boxes_two[0])
                if abs(cx - 200) < 100 and 0 < area_two < 7000:
                    msg = "linh"+"_"+"t"+"\n"
                elif (cx - 200) > 100:
                    msg = "linh"+"_"+"h"+"\n"
                elif (cx - 200) < -100:
                    msg = "linh"+"_"+"f"+"\n"
            else:
                # flag_move = True
                ctrcnst = ['f','t','g','h','o']
                if count % 3 == 0:
                    ctr = np.random.randint(4)
                    count=0
                count+=1
                msg = "linh"+"_"+ctrcnst[ctr]+"\n"

            print(flag_move, msg)
            if flag_move == True:
                try:
                    if state_stop == False:
                        idx = 0
                        for i in start[idx]:
                            msg += '0'*(3-len(str(i))) + str(i)
                        msg += '\n'
                    sv.send(msg.encode())
                    time.sleep(1)
                except:
                    sv.close()
                    reconect = True
            elif flag_move == False:
                if state_stop == True:
                    state_stop = False
                    msg = "linh"+"_"+"o"+"\n"
                    sv.send(msg.encode())
                    time.sleep(1.5)
                if idx == 2:
                    angles = angles_current
                    angles[1] -= 2
                    angles[2] -= 2
                    angles[3] = 77
                elif idx == 3:
                    angles = angles_current
                    angles[3] = 20
                else:
                    angles = start[idx]
                # angles = angles_current
                # angles[2] -= 2
                # angles[3] = 77

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
                    time.sleep(3)
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
                    sv.close()
                    time.sleep(5)
                # break
        else:
            print("Cam left connect: ", check_cam_left)
            print("Cam right connect: ", check_cam_right)


if __name__ == '__main__':
    print("Started")

    # t = time.time()
    t1 = threading.Thread(target=cam_left, args=(url1,))
    t2 = threading.Thread(target=cam_right, args=(url2,))
    # t3 = threading.Thread(target=auto_robot_v2)
    t4 = threading.Thread(target=control_robot)

    t1.start()
    t2.start()
    # t3.start()
    t4.start()

    # t1.join()
    # t2.join()
    # t3.join()
    # t4.join()
