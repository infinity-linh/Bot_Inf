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
from model_CNN import Net


# url1 = 'http://192.168.2.103/cam-lo.jpg'
# url2 = 'http://192.168.2.102/cam-lo.jpg'
url1 = 'http://192.168.1.7/cam-lo.jpg'
url2 = 'http://192.168.1.9/cam-lo.jpg'


PATH = 'D:/User/DLBot/scripts/model/move_model_new_256.pt'

if torch.cuda.is_available():
    device = "cuda:0"
else:
    device = "cpu"

detect = Detection()

net = Net()
net.load_state_dict(torch.load(PATH, map_location=torch.device(device)))

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
    count = 0
    while True:
        if check_cam == False:
            cv2.namedWindow('Camera l:', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Camera r:', cv2.WINDOW_NORMAL)

        try:
            img_resp1 = urllib.request.urlopen(url1, timeout=2)
            img_resp2 = urllib.request.urlopen(url2, timeout=2)

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


            key = cv2.waitKey(1)
            if key == 32:
                cv2.imwrite('D:/User/data_object/images/imagel_{}.png'.format(count), image_eye_rootl)
                cv2.imwrite('D:/User/data_object/images/imager_{}.png'.format(count), image_eye_rootr)
                count+=1


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
                    xx = random.randint(7, 18)
                    yy = random.randint(-7, 4)
                    zz = random.randint(2, 5)
                    print(xx, yy, zz)
                    angles_base, angles_show, angles_elbo = donghocnguoc(xx, yy, zz)  
                    # angles_base = random.randint(80, 120)
                    # angles_show = random.randint(20, 60)
                    # angles_elbo = random.randint(50, 80)
                    angles_grip = random.randint(15, 20)

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
    path_csv = "D:/User/data_map/locals.csv"
    direct = [104,103,102,116]
    path_save_l = "D:/User/data_map/left/"
    path_save_r = "D:/User/data_map/right/"
    idx_name = len(os.listdir(path_save_l))
    print(idx_name)
    # cv2.namedWindow("Eye:", cv2.WINDOW_NORMAL)
    numofimage = 0
    auto = False
    # state_study = False
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

            # try:
            if key != -1:
                if key == 32:
                    auto = ~auto
                    print("Auto: ", auto)
                # elif key == 27:
                #     study = ~state_study
                # if auto!=True:f
                else:
                # if key!=32:

                    data_frame = []
                    df = pd.read_csv(path_csv)

                    name_l = "imagel_{}.png".format(idx_name)
                    name_r = "imager_{}.png".format(idx_name)
                    data_out, send_msg, _ = control_v(sv, key)
                        
                    # data_pre = data_outtf
                    # if data_out != 't':
                    data_frame.append(name_l)
                    data_frame.append(name_r)
                    data_frame.append(data_out)
                    df2 = pd.DataFrame([data_frame], columns=[
                                    'imagel', 'imager', 'action'])
                    df2 = pd.concat([df, df2])
                    # if numofimage > 99:
                    #     sv.close()
                    #     break
                    # if data_out == 'h' or data_out == 'f' or data_out == 'g':
                    cv2.imwrite(path_save_l+name_l, image_eye_rootl)
                    cv2.imwrite(path_save_r+name_r, image_eye_rootr)
                    df2.to_csv(
                        path_csv, index=False)
                    numofimage+=1
                    print("Number of {}: {}".format(data_out, numofimage))

                    # print(data_frame)
                    idx_name += 1
                    # time.sleep(2)
                    try:
                        print(send_msg)
                        sv.send(send_msg.encode())
                    except:
                        sv.close()
                        reconect = True
            if auto == -1: 
                if len(image_eye_rootl)!=0 and len(image_eye_rootr) != 0:
                    image1 = cv2.resize(image_eye_rootl, [128,256])
                    image2 = cv2.resize(image_eye_rootr, [128,256])
                    image = cv2.hconcat([image1, image2]).transpose([2,0,1])
                    # image1, image2 = torch.tensor(np.array([image1]),dtype=torch.float32), 
                    image = torch.tensor(np.array([image]),dtype=torch.float32)
                    outputs = net(image)
                    _, predicted = torch.max(outputs, 1)
                    sig = np.array(predicted, dtype=np.uint8)[0]
                    print(direct[sig])
                    dta, send_msg, _ = control_v(sv, direct[sig])
                    print("Direct: ",dta)
                    # if state_study == -1:

                try:
                    print(send_msg)
                    sv.send(send_msg.encode())
                    time.sleep(1)
                except:
                    sv.close()
                    reconect = True
            # except:
            #     sv.close()
            #     reconect = True
        else:
            sv.close()
            cv2.destroyWindow("Eye:")
            print("Status camera:", check_cam)
            reconect = True

def colection_data():
    path_save_l = "D:/User/data_object/images/"
    idx_name = len(os.listdir(path_save_l))
    while True:
        if check_cam == True:
            try:
                name_l = "imagel_{}.png".format(idx_name)
                print(name_l)
                cv2.imwrite(path_save_l+name_l, image_eye_rootl)
                idx_name+=1
                time.sleep(3)
            except:
                pass

def test_arm ():
    global ag
    PORT = 8090
    reconect = True

    SERVER = socket.gethostbyname(socket.gethostname())
    ADDR = (SERVER, PORT)
    print(SERVER)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(ADDR)
    server.listen()
    model_ctr = ANN(8)
    msg = ''
    model_ctr.load_state_dict(torch.load("D:/User/DLBot/scripts/model/model_auto_arm.pt"))
    while True:
        if reconect == True:
            sv, addr = server.accept()
            reconect = False
        if len(boxes_one) != 0 and len(boxes_two) != 0:

            boxes = np.array([
                    np.hstack([boxes_one[0], boxes_two[0]])/400], dtype=np.float32)
        
            angles = model_ctr(torch.tensor(
            boxes, dtype=torch.float32))
            angles = np.array(angles.detach().numpy()*180/np.pi, dtype=np.int8)[0]

            print(angles)
            try:
                for i in angles:
                    msg += '0'*(3-len(str(i))) + str(i)
                msg += '\n'
                # if state_stop == False:
                print(msg)
                sv.send(msg.encode())
                time.sleep(1)
            except: 
                reconect = False
                time.sleep(1)
                pass
if __name__ == '__main__':
    print("Started")

    # t = time.time()
    t1 = threading.Thread(target=eyes, args=(url1, url2,))
    # t2 = threading.Thread(target=create_data)
    t2 = threading.Thread(target=auto_robot_v2)
    # t2 = threading.Thread(target=colection_data)
    # t2 = threading.Thread(target=test_arm)



    t1.start()
    t2.start()

