#! /usr/bin/env python3
import concurrent.futures
import serial
import time
import cv2
import numpy as np
import urllib.request
from detector import Detection
from control_angle import *
import socket
import pandas as pd

# url='http://192.168.2.103/cam-hi.jpg'
# url='http://192.168.2.103/cam-mid.jpg'
url1 = 'http://192.168.2.110/cam-lo.jpg'
url2 = 'http://192.168.2.108/cam-lo.jpg'
detect = Detection()
state_one = [106,169,58,10, 255, 255]

def calcula(state, direct, type):
    if type == True:
        if state > 254 or state < 1:
            return state
        else:
            state = state + direct
        return state
    else :
        if state > 179 or state < 1:
            return state
        else:
            state = state + direct
        return state

def calculator(data_input):
    if data_input == "a":
        # state_one[0] -= 1
        calcula(state_one[0], -1, False)
    elif data_input == "d":
        # state_one[0] += 1
        calcula(state_one[0], 1, False)

    if data_input == "w":
        # state_one[1] -= 1
        calcula(state_one[1], -1, False)
    elif data_input == "s":
        # state_one[1] += 1
        calcula(state_one[1], 1, False)

    if data_input == "u":
        # state_one[2] -= 1
        calcula(state_one[2], -1, False)
    elif data_input == "b":
        # state_one[2] += 1
        calcula(state_one[2], 1, False)

    if data_input == "l":
        # state_one[3] -= 1
        calcula(state_one[3], -1, False)
    elif data_input == "r":
        # state_one[3] += 1
        calcula(state_one[3], 1, False)


    if data_input == "i":
        # state_one[4] -= 1
        calcula(state_one[4], -1, True)

    elif data_input == "j":
        # state_one[4] += 1
        calcula(state_one[4], 1, True)

    
    if data_input == "k":
        # state_one[5] -= 1
        calcula(state_one[5], -1, True)

    elif data_input == "p":
        # state_one[5] += 1
        calcula(state_one[5], 1, True)

    return state_one

def control_keyboard(port, key):
    msg = {32: "o",
           97: "a",
           119: "w",
           115: "s",
           100: "d",
           50: "u",
           56: "b",
           52: "l",
           54: "r",
           116: 't',
           102: 'h',
           104: 'f',
           103: 'g',
           49:  'i',
           51: 'k',
           55: 'j',
           57:'p'}
    send_msg = "linh"+"_"+str(msg[key])+"\n"
    port.send(send_msg.encode())
    return calculator(str(msg[key]))


def change_brightness(img, alpha, beta):
    # cast pixel values to int
    img_new = np.asarray(alpha*img + beta, dtype=int)
    img_new[img_new > 255] = 255
    img_new[img_new < 0] = 0
    return np.array(img_new, dtype=np.uint8)


def auto_robot():
    SETUP = False
    port = None
    prev = time.time()
    while (not SETUP):
        try:
            port = serial.Serial("COM4", 115200, timeout=1)

        except:  # Bad way of writing excepts (always know your errors)
            if (time.time() - prev > 2):  # Don't spam with msg
                print("No serial detected, please plug your uController")
                prev = time.time()

        if (port is not None):  # We're connected
            SETUP = True
    print("I'm ready.")
    start = [[106, 169, 58, 10],
             [106, 169, 58, 10],
             [108, 100, 100, 77],
             [108, 48, 166, 10],
             [108, 135, 100, 10],
             [108, 100, 60, 10],
             [23, 140, 39, 10],
             [23, 140, 39, 100],
             [106, 169, 58, 10],
             ]

    # move_start = ['t', 'h', 'f', 'g', 'o']
    # for i in range(len(move_start)):
    #     move_location(port, move_start[i])
    for i in range(len(start)-1):
        move_point(port, start[i], start[i+1], idx=0, direct=1)

def auto_robot_v1():
    PORT = 8090
    SERVER = socket.gethostbyname(socket.gethostname())
    ADDR = (SERVER, PORT)
    print(SERVER)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(ADDR)
    server.listen()
    c, addr = server.accept()

    start = [[106, 169, 58, 10],
             [106, 169, 58, 10],
             [108, 100, 100, 77],
             [108, 48, 166, 10],
             [108, 135, 100, 10],
             [108, 100, 60, 10],
             [23, 140, 39, 10],
             [23, 140, 39, 100],
             [106, 169, 58, 10],
             ]

    # move_start = ['t', 'h', 'f', 'g', 'o']
    # for i in range(len(move_start)):
    #     move_location(port, move_start[i])
    for i in range(len(start)-1):
        move_point(c, start[i], start[i+1], idx=0, direct=1)

def auto_robot_v2():

    PORT = 8090
    SERVER = socket.gethostbyname(socket.gethostname())
    ADDR = (SERVER, PORT)
    print(SERVER)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(ADDR)
    server.listen()
    cv2.namedWindow("EYES: ", cv2.WINDOW_AUTOSIZE)
    frame = cv2.imread("D:/User/firmware/Screen/hinh-anh-mat-cuoi2-1.png")
    c, addr = server.accept()
    print('Got connection from', addr)

    while True:
        # _, frame = cam.read()
        cv2.imshow("EYES: ", frame)
        key = cv2.waitKey(1)

        if key != -1:
            print(key)
            try:
                data_out = control_keyboard(c, key)
                print(data_out)
                # time.sleep(0.1)
            except:
                # print('err')
                pass
        if key == ord('x'):
            c.close()
            break
            # c.close()
    cv2.destroyAllWindows()


def run():
    SETUP = False
    port = None
    prev = time.time()
    while (not SETUP):
        try:
            port = serial.Serial("COM4", 115200, timeout=1)

        except:  # Bad way of writing excepts (always know your errors)
            if (time.time() - prev > 2):  # Don't spam with msg
                print("No serial detected, please plug your uController")
                prev = time.time()

        if (port is not None):  # We're connected
            SETUP = True

    cv2.namedWindow("EYES: ", cv2.WINDOW_AUTOSIZE)
    frame = cv2.imread("D:/User/firmware/Screen/hinh-anh-mat-cuoi2-1.png")
    while True:
        # _, frame = cam.read()

        cv2.imshow("EYES: ", frame)
        key = cv2.waitKey(1)
        if key != -1:
            # print(key)
            try:
                control_keyboard(port, key)
                pd 
            except:
                # print('err')
                pass
        if key == ord('x'):
            break
    cv2.destroyAllWindows()


def run_camera_right():
    print("Camera one ready!")
    while True:
        # _, frame = cam.read()
        try:
            img_resp = urllib.request.urlopen(url1)
            imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
            image_eye_left = cv2.imdecode(imgnp, 1)
            image_eye_left = change_brightness(image_eye_left, 1.0, 35)
            image_eye_left = cv2.rotate(
                image_eye_left, cv2.ROTATE_90_CLOCKWISE)
            image_eye_left = cv2.resize(image_eye_left, (400, 400))
            # eyes = cv2.hconcat([frame,frame])
            boxes, image_eye_left = detect.detector_image(image_eye_left)
            print(boxes)
            cv2.imshow("Camera right: ", image_eye_left)
        except:
            cv2.destroyAllWindows()
            continue
        key = cv2.waitKey(1)
        if key == ord('x'):
            break
    cv2.destroyAllWindows()


def run_camera_left():
    count = 95
    print("Camera two ready!")
    while True:
        try:
            img_resp = urllib.request.urlopen(url2)
            imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
            image_eye_right = cv2.imdecode(imgnp, 1)
            # image_eye_right = cv2.flip(image_eye_right, -1)
            image_eye_right = change_brightness(image_eye_right, 1.0, 35)
            image_eye_right = cv2.rotate(
                image_eye_right, cv2.ROTATE_90_CLOCKWISE)
            image_eye_right = cv2.resize(image_eye_right, (400, 400))
            image_eye_right = detect.detector_image(image_eye_right)
            cv2.imshow("Camera left: ", image_eye_right)
        except:
            cv2.destroyAllWindows()
            continue
        key = cv2.waitKey(1)
        if key == ord('s'):
            cv2.imwrite(
                "D:/User/ESP32_DOIT/data/image/frame_{}.png".format(count), image_eye_right)
            print("Save image frame_{} in folder: /data/image".format(count))
            count += 1
        if key == ord('x'):
            break
    cv2.destroyAllWindows()


def run_eyes():
    num = 1
    while True:

        # _, frame = cam.read()
        try:
            img_resp_1 = urllib.request.urlopen(url1)
            # img_resp_2 = urllib.request.urlopen(url2)

            imgnp_1 = np.array(bytearray(img_resp_1.read()), dtype=np.uint8)
            # imgnp_2 = np.array(bytearray(img_resp_2.read()), dtype=np.uint8)

            image_eye_left_1 = cv2.imdecode(imgnp_1, 1)
            # image_eye_left_2 = cv2.imdecode(imgnp_2, 1)

            image_eye_left_1 = change_brightness(image_eye_left_1, 1.0, 35)
            # image_eye_left_2 = change_brightness(image_eye_left_2, 1.0, 35)

            image_eye_left_1 = cv2.rotate(
                image_eye_left_1, cv2.ROTATE_90_CLOCKWISE)
            # image_eye_left_2 = cv2.rotate(
            #     image_eye_left_2, cv2.ROTATE_90_CLOCKWISE)

            image_eye_left_1 = cv2.resize(image_eye_left_1, (400, 400))
            # image_eye_left_2 = cv2.resize(image_eye_left_2, (400, 400))

            # image_eye_left_1 = detect.detector_image(image_eye_left_1)
            # image_eye_left_2 = detect.detector_image(image_eye_left_2)
            cv2.imwrite("D:/User/data/image/val/image_{}.png".format(num),image_eye_left_1)
            # cv2.imwrite("D:/User/data/image/val/image_{}.png".format(num+1),image_eye_left_2)
            num+=1
            if num>5:
                break
            # eyes = cv2.hconcat([image_eye_left_1, image_eye_left_2])
            cv2.imshow("Camera right: ", image_eye_left_1)

        except:
            cv2.destroyAllWindows()
            continue
        key = cv2.waitKey(1)
        if key == ord('x'):
            break
    cv2.destroyAllWindows()


if __name__ == '__main__':
    print("started")
    # run_camera_left()

    with concurrent.futures.ProcessPoolExecutor(max_workers=5) as executer:
        # funzero = executer.submit(run)
        # funzero = executer.submit(auto_robot_v2)
        # funzero = executer.submit(auto_robot_v1)

        # funone = executer.submit(run_camera_right)
        # funtwo = executer.submit(run_camera_left)
    # auto_robot_v2()
        funfour = executer.submit(run_eyes)



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
        flag_move == True

        if reconect == True:
            sv, addr = server.accept()
            reconect = False
            print ('Got connection from', addr )

        if len(boxes_one) != 0 and len(boxes_two) != 0 :
        # and flag_move==True:
            # boxes = np.array([np.hstack([center_box(boxes_one[0]), center_box(boxes_two[0])])/400],dtype=np.float32)
            boxes = np.array(np.hstack([boxes_one[0], boxes_two[0]])/400,dtype=np.float32)
            
            angles = model_ctr(torch.tensor(boxes, dtype=torch.float32))*180
            angles = np.array(angles.detach().numpy()[0], dtype=np.int8)
            angles_current = angles
            flag_move = False
            print("Angles: ", angles_current)
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
            if idx == 3:
                angles = angles_current
                angles[3] = 77
            else:
                angles = start[idx]
            
            msg = ''
            if angles[0] >= 0 and angles[1] >= 0 and angles[2] >= 0 and angles[3] >= 0:
                angles = angles
            else:
                angles = start[idx]
            for i in angles:
                msg += '0'*(3-len(str(i))) + str(i)
            msg+='\n'
            
            # print(idx, len(start)," Box_one: ", boxes_one," Box_two: ", boxes_two, "Angles: ", angles, "Msg: ", msg)
        # print(sv.recv(3))
            # sv.send(msg.encode())
        # if sv.recv(3) == b'sta':
        # try:
        sv.send(msg.encode())
        time.sleep(3)
        # print('start')
        if reconect == False and sv.recv(3)==b'end':
            idx+=1
        # except:
            # sv.close()
            # reconect = True
        if idx >= len(start):
            print(idx, "Stop")
            idx = 0
            reconect = True
            sv.close()
            # break