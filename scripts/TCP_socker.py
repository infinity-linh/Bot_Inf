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

while True:
    # data

    print ('Got connection from', addr )

    # msg = "linh"+"_"+"o"+"\n"

    msg = "180090090090"+"\n"
    # send a thank you message to the client. encoding to send byte type.
    c.send(msg.encode())

    # Close the connection with the client
    time.sleep(3.5)
    # Breaking once connection closed
    # break
    c.close()

