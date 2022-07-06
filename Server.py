import socket

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "192.168.1.180"
port = 30000
print (host)
print (port)
serversocket.bind((host, port))

serversocket.listen(5)
print ('server started and listening')

(clientsocket, address) = serversocket.accept()

while True:
    data = clientsocket.recv(1024).decode()
    print (data)
