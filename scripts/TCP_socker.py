import socket, time

PORT = 8090
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = (SERVER, PORT)
print(SERVER)
FORMAT = "utf-8"
DISCONNECT_MESSAGE = "!DISCONNECT"


server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# print(server)
server.bind(ADDR)
server.listen()
c, addr = server.accept()    
angles = [100, #80-100
          60, #35 - 60
          100, #50-100
          0]
count = 0
while True:
    # data
    msg = '' 
    print ('Got connection from', addr )

    # msg = "255255"+"t"+"\n"
    for st in angles:
        msg += '0'*(3-len(str(st))) + str(st)
    msg += '\n' 
    # msg = "106048100090"
    # send a thank you message to the client. encoding to send byte type.
    c.send(msg.encode())
    # print(msg)

    # Close the connection with the client
    time.sleep(3)
    count += 1
    if count > 2:
        break
    # Breaking once connection closed
    # break

c.close()

