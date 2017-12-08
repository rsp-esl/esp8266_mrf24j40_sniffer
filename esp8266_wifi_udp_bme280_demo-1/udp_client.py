#!/usr/bin/env python

import socket
import time

server_ip = "127.0.0.1"
remote_port = 8888

print ( "UDP target IP: {:s}".format( server_ip ) )
print ( "UDP target port: {:d}".format( remote_port ) )

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

count = 1
try:
    while True:
        msg = "Hello #{:04d}\n".format( count )
        count = count+1
        sock.sendto( msg, (server_ip,remote_port) )
        data, addr = sock.recvfrom(1024)
        if data:
            print ( str(data).strip() )
        time.sleep(1.0)

except (KeyboardInterrupt):
    print ('Terminated...')

##########################################################################
