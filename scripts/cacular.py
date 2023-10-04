
# import cv2
import torch
import numpy as np
state_one = [106, 169, 58, 10]


def control_keyboard(port, key):
    msg = {32: "o",
           #    97: "a",
           #    119: "w",
           #    115: "s",
           #    100: "d",
           #    50: "u",
           #    56: "b",
           #    52: "l",
           #    54: "r",
           116: 't',
           102: 'h',
           104: 'f',
           103: 'g'}

    send_msg = "255350"+str(msg[key])+"\n"
    port.send(send_msg.encode())
    return str(msg[key])
# return 0


def control_v(port, key):
    msg = {32: "o",
           116: 't',
           102: 'h',
           104: 'f',
           103: 'g'}
    speed = {32: '000',
           116: '500',
           102: '150',
           104: '150',
           103: '500'}
    send_msg = "250"+speed[key]+str(msg[key])+"\n"
    # print(send_msg)
    # port.send(send_msg.encode())
    return str(msg[key]), send_msg


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


def square_box(box):
    return (box[2]-box[0]) * (box[3]-box[1])


def convert_map(angle, inmax, inmin, outmax, outmin):
    return (angle-inmin)*(outmax-outmin)/(inmax-inmin) + outmin


def cacular_speed(areas):
    if areas < 400:
        different = abs(areas - 200)
        # if different > 0:
        # print(different)
        speed = convert_map(different, 300, 0, 200, 0)
        # print("angles:",speed)
        v = '0'*(3-len(str(int(speed)))) + str(int(speed))
        return '255'+v
        # return '230230'
    else:
        different = 15000 - areas
        print("Diff: ", different)
        if different > 0:
            speed = convert_map(different, 15000, 0, 400, 20)
            # print(speed)
            v = '0'*(3-len(str(int(speed)))) + str(int(speed))
            return '255'+v
    return '255200'

def state_robot(boxes_one, boxes_two, model_ctr, flag_move, state_stop, catch_com, angles_current, state_move, idx):
    speed = "255150"
    msg = ''
    area_one, area_two = 0, 0
    print(boxes_one, boxes_two)
    if len(boxes_one) != 0 and len(boxes_two) != 0:
        # and flag_move==True:
        # print(boxes_one, boxes_two)
        area_one, area_two = square_box(
            boxes_one[0]), square_box(boxes_two[0])
        cox, coy = center_box(boxes_one[0])
        ctx, cty = center_box(boxes_two[0])
        # print(area_one, area_two)
        if 15000 < area_one < 30000 or 15000 < area_two < 30000:
            if abs(cox - 200) < 120 and 0 < ctx - 200 < 150:
                # boxes = np.array([np.hstack([center_box(boxes_one[0]), center_box(boxes_two[0])])/400],dtype=np.float32)
                boxes = np.array(
                    np.hstack([boxes_two[0], boxes_one[0]])/400, dtype=np.float32)

                angles = model_ctr(torch.tensor(
                    boxes, dtype=torch.float32))*180
                angles = np.array(angles.detach().numpy(), dtype=np.int8)
                angles_current = angles
                flag_move = False
                state_stop = True
                catch_com = False
                state_move = False
    else:
        state_move = True

    if (len(boxes_one) != 0):
        state_move = False
        area_one = square_box(boxes_one[0])
        cox, coy = center_box(boxes_one[0])

        if abs(cox - 200) > 100:
            speed = cacular_speed(cox)

        if abs(cox - 200) < 100:
            speed = cacular_speed(area_one)
            msg = speed+"t"+"\n"
        elif (cox - 200) > 100:
            msg = speed+"f"+"\n"
        elif (cox - 200) < -100:
            msg = speed+"h"+"\n"

        # if area_one < 10000:
        #     msg = speed+"t"+"\n"
        if area_one > 20000:
            msg = speed+"g"+"\n"

    elif (len(boxes_two) != 0):
        state_move = False
        area_two = square_box(boxes_two[0])
        ctx, cty = center_box(boxes_two[0])

        if abs(ctx - 200) > 100:
            speed = cacular_speed(ctx)
        if abs(ctx - 200) < 100:
            msg = speed+"h"+"\n"
        elif (ctx - 200) > 100:
            msg = speed+"t"+"\n"
        elif (ctx - 200) < -100:
            msg = speed+"g"+"\n"
        # if area_two < 10000:
        #     msg = speed+"t"+"\n"
        if area_two > 20000:
            msg = speed+"g"+"\n"

    elif (catch_com == True):
        msg = speed+"o"+"\n"
        state_move = True
    else:
        msg = speed+"h"+"\n"
        idx+=1
    if idx>5:
        idx = 0
        state_move = True

    return msg, flag_move, state_stop, catch_com, angles_current, state_move, idx
