import socket
import sys
import pygame
import pygame.camera


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = ('192.168.0.105', 10000)
print >>sys.stderr, 'starting up on %s port %s' % server_address
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)
i=0
while True:
    # Wait for a connection
    print >>sys.stderr, 'waiting for a connection'
    connection, client_address = sock.accept()
    try:
        print >>sys.stderr, 'connection from', client_address

        # Receive the data in small chunks and retransmit it
        while True:
            data = connection.recv(16)
            print >>sys.stderr, 'received "%s"' % data
            if data == "foto":
                i += 1
                pygame.camera.init()
                pygame.camera.list_cameras()
                cam = pygame.camera.Camera("/dev/video0",(640,480))
                cam.start()
                img = cam.get_image()
                filename = "filename%d.jpg" % i
                pygame.image.save(img,filename)

                print >>sys.stderr, 'sending data back to the client'
                connection.sendall(filename)
            elif data:
                print >>sys.stderr, 'sending data back to the client'
                connection.sendall(data)
            else:
                print >>sys.stderr, 'no more data from', client_address
                break
            
    finally:
        # Clean up the connection
        connection.close()
