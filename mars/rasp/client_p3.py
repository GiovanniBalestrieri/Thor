import socket
import sys

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('192.168.0.103', 40000)
print('connecting to %s port %s' % server_address, file=sys.stderr)
sock.connect(server_address)

try:
    
    # Send data
    message = 'obstacles'
    print('sending "%s"' % message, file=sys.stderr)


    # f = open('filename2.jpg','rb')
    # datix = f.read()
    #datix = b'0'*20000
    # print(len(datix))
    # f.close()

    sock.sendall(message.encode())

    # Look for the response
    amount_received = 0
    amount_expected = len(message)
    
    while True:
        data = sock.recv(16)
        if not data:
            break
        amount_received += len(data)
        print('received "%s"' % data, file=sys.stderr)

finally:
    print('closing socket', file=sys.stderr)
    sock.close()
