
# import cv2
state_one = [106, 169, 58, 10]


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
           103: 'g'}
    try:
        send_msg = "linh"+"_"+str(msg[key])+"\n"
        port.send(send_msg.encode())
# return 0
        return calculator(str(msg[key]))
    except:
        
        return None

def calcula(state, direct, type):
    if type == False:
        posstion = state_one[state]
        posstion = posstion + direct
        if posstion > 180 or posstion < 0:
            return
        state_one[state] = posstion
    else:
        posstion = state_one[state]
        posstion = posstion + direct
        if posstion > 255 or posstion < 0:
            return
        state_one[state] = posstion


def calculator(data_input):
    if data_input == "a":
        # state_one[0] -= 1
        calcula(0, -1, False)
    elif data_input == "d":
        # state_one[0] += 1
        calcula(0, 1, False)

    if data_input == "w":
        # state_one[1] -= 1
        calcula(1, -1, False)
    elif data_input == "s":
        # state_one[1] += 1
        calcula(1, 1, False)

    if data_input == "u":
        # state_one[2] -= 1
        calcula(2, -1, False)
    elif data_input == "b":
        # state_one[2] += 1
        calcula(2, 1, False)

    if data_input == "l":
        # state_one[3] -= 1
        calcula(3, -1, False)
    elif data_input == "r":
        # state_one[3] += 1
        calcula(3, 1, False)

    if data_input == "i":
        # state_one[4] -= 1
        calcula(4, -1, True)

    elif data_input == "j":
        # state_one[4] += 1
        calcula(4, 1, True)

    if data_input == "k":
        # state_one[5] -= 1
        calcula(5, -1, True)

    elif data_input == "p":
        # state_one[5] += 1
        calcula(5, 1, True)

    return state_one

def center_box(box):
    x, y = box[0]+(box[2] - box[0])//2, box[1]+(box[3] - box[1])//2 
    return x, y
# frame = cv2.imread("D:/User/firmware/Screen/hinh-anh-mat-cuoi2-1.png")

# while True:
#     # _, frame = cam.read()
#     cv2.imshow("EYES: ", frame)
#     key = cv2.waitKey(1)

#     if key != -1:
#         # print(key)
#         try:
#             data_out = control_keyboard(key)
#             print(data_out)
#             # time.sleep(0.1)
#         except:
#             # print('err')
#             pass
#     if key == ord('x'):
#         break
#         # c.close()
# cv2.destroyAllWindows()
