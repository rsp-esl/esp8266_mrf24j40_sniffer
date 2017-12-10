#!/usr/bin/env python
# -*- coding: utf-8 -*-

###########################################################################
# Author: Rawat S. 
#         Department of Electrical & Computer Engineering (ECE)
#         Faculty of Engineering, KMUTNB, Thailand
#
# Date: 2017-12-11
###########################################################################

import socket
import binascii 
import time
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.settimeout(2.0) # set timeout for 2.0 seconds

server_address = '0.0.0.0'
server_port = 8888

server = (server_address, server_port)
sock.bind(server)

print("Listening on " + server_address + ":" + str(server_port))

cnt = 0
ts1 = int(round(time.time() * 1000))

while True:
    try:
        # wait for the next incoming UDP packet
        try:
            payload, client_address = sock.recvfrom(512) 
        except socket.timeout:
            payload = None

        ts2 = int(round(time.time() * 1000))

        if payload:
            ts1 = ts2 
    	    num_bytes = ord(payload[0])
    	    frame_data = payload[1:num_bytes-2]
    	    lqi  = ord(payload[-2])
    	    rssi = ord(payload[-1])
    	    frame_data_hex = binascii.hexlify( payload[1:-2] ).decode('ascii') 
    	    print( "{:d}) {:s},{:02x},{:02x}".format( cnt+1, frame_data_hex, lqi, rssi ) )
    	    if num_bytes != len(payload):
    	        print( 'data length error !!!' )
    	    sys.stdout.flush()
            resp = 'OK: {:d}\n'.format(cnt)
            sent = sock.sendto(resp, client_address)

        if ts2 - ts1 >= 2000:
            if cnt > 0:
                print (40*'=')
                print ( 'No. of captured packets: {:d}'.format(cnt) )
                print (40*'=')
            else:
                print ('.')
            sys.stdout.flush()
            ts1 = ts2
            cnt = 0
        else:
        	cnt = cnt+1

    except KeyboardInterrupt:
    	print ('Terminated...')

sock.close()

###########################################################################
