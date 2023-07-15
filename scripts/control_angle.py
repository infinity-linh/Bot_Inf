def move_point(port, input, output, idx=0, direct=1):
    msg = ["a","d","w","s","u","b","l","r"]
    step = 1
    start = input.copy()
    end = output.copy()
    data = start[idx] - end[idx]
    if data == 0:
        idx += direct 
    if data < 0:
        # print(msg[2*idx])
        print(port.read(1))
        # port.write(msg[2*idx+1].encode())
        send_msg = "linh"+"_"+str(msg[2*idx+1])+"\n"

        port.send(send_msg.encode())
        time.sleep(0.1)
        start[idx] = start[idx] + step
    elif data > 0:
        # print(msg[2*idx+1])
        # port.write(msg[2*idx].encode())
        send_msg = "linh"+"_"+str(msg[2*idx])+"\n"

        port.send(send_msg.encode())

        time.sleep(0.1)

        start[idx] = start[idx] - step
    if idx > 3 or idx < 0:
        return 
    # print(idx, start, end) 
    move_point(port, start, end, idx, direct)
import time



def move_location(port, input):
    # port.write(input.encode())
    
    time.sleep(1)


def convert_msg(data, direct):
    msg = ''
    if direct < 0:
        data = data[::-1]
    else:
        data = data
    for d in data:
        msg += (3-len(str(d)))*'0' + str(d)
    return msg

# def move_point(port, start, end, idx=0, direct=1):
#     step = 10
#     loca = start.copy()
#     data = start[idx] - end[idx]
#     if data == 0:
#         idx += direct 
#     if data < 0:
#         start[idx] = start[idx] + step
#         print(convert_msg(start, direct))
#         port.write(convert_msg(start, direct).encode())
#         # time.sleep(1)
#     elif data > 0:
#         start[idx] = start[idx] - step
#         print(convert_msg(start, direct))
#         port.write(convert_msg(start, direct).encode())
#         # time.sleep(1)

#     if idx > 3 or idx < 0:
#         return 
#     # print(idx, start, end) 
#     move_point(port, start, end, idx, direct)
#     time.sleep(1)

def step_move(port, start, end, dir):
    value = end.copy()
    # value = value
    # print(end)
    if dir < 0:
        value[0] = start[0]
        value[1] = start[1]
        value[2] = start[2]
        port.write(convert_msg(value, 1).encode())
        time.sleep(1)

    print(end)
    port.write(convert_msg(end, 1).encode())
    time.sleep(1)



# port = None
# start = [106,169,58,10]
# end = [23,135,39,10]
# step_move(port,start, end,-1)

# def move_location(port, point, direct=1):
#     print(point)
#     port.write(convert_msg(point, direct).encode())

    