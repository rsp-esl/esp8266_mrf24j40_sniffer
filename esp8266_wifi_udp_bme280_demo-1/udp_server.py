#!/usr/bin/env python
# -*- coding: utf-8 -*-

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_address = '0.0.0.0'
server_port = 8888

server = (server_address, server_port)
sock.bind(server)

print("Listening on " + server_address + ":" + str(server_port))

cnt = 0
while True:
    payload, client_address = sock.recvfrom(255)
    print("Packet received, sending the response back... " + str(client_address))
    if payload:
        print (str(payload).strip() )
        response = 'OK: {:d}\n'.format(cnt)
        sent = sock.sendto(response, client_address)
        print ('response: ' + response.strip() )
        cnt = cnt+1

#########################################################################3
