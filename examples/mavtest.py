#!/usr/bin/env python

"""
Generate a message using different MAVLink versions, put in a buffer and then read from it.
"""




from __future__ import print_function
from builtins import object

from pymavlink.dialects.v10 import common as mavlink1
from pymavlink.dialects.v20 import common as mavlink2

from pymavlink import mavutil

import time
import serial

'''
Buffer Class fifo
'''

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

'''
Festlegen des Ports
'''

s_port = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate = 57600,
    parity = serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
'''
Funktion zum Lesen des Ports. Uebergibt eine list mit Hexadezimalzahlen
'''
def read_port():
    out = []
    while s_port.inWaiting() > 0:
        new_data = s_port.read(1)
        out += new_data  # pure number string

        # teststring = '-2000\r\np\r\nf\r\nOK\r\n'
        # out = teststring

        #time.sleep(0.05)

    #out_split = out.rstrip().split('\r\n')
    return out

def write_on_port(strcommand):
    s_port.write(strcommand + '\r\n')
    time.sleep(0.05)
    return True



print("Creating MAVLink message...")
# create a mavlink instance, which will do IO on file object 'f'
'''

s_port.isOpen()

i=0
while i < 255:
    f = fifo()

    #mav ist die Mavlink Message. Hier in mavlink v1
    mav = mavlink1.MAVLink(f)

    #out ist eine Liste mit hexadezimalen Zahlen
    out = read_port()
    time.sleep(0.05)
    if out == []:
        continue
    print (out)
    #bi ist eine Liste mit arabischen Zahlen
    bi = []
    bi = ([ord(c) for c in out])

    #Diese Schleife ordnet bi[] so, dass bi[] immer mit einem Nachrichtenbeginn startet (bi[0] = 254)
    while bi[0]!=254:
        del bi[0]
        if bi==[]:
            break
        #print('bi = ' + str(bi))

    if bi==[]:
        continue

    #stellt sicher, dass in bi[] nur eine Nachricht enthalten ist
    if (len(bi)>(bi[1]+8) and bi!=[]):
        del bi[1+bi[1:].index(254):]

    #print('bi_len ' + str(len(bi)))
    #if len(bi) < 10:
    #    continue

    if len(bi)==(bi[1]+8):
        print('bi = ' + str(bi))
        #headerlen = 6
        #print('header' + str(bi[:headerlen]))
        bii = ([hex(d) for d in bi])
        #bii = ([chr(d) for d in bi])
        print(bii[0]=='0xfe')
        print(mav.decode(bii))
        #if bi[0] == '254':
        #    print(mav.decode(bi))
        print('\n')
    #"""
    i+=1

s_port.close()

'''


def test_protocol(mavlink, signing=False):
    # we will use a fifo as an encode/decode buffer
    f = fifo()

    print("Creating MAVLink message...")
    # create a mavlink instance, which will do IO on file object 'f'
    mav = mavlink.MAVLink(f)


    if signing:
        mav.signing.secret_key = chr(42)*32
        mav.signing.link_id = 0
        mav.signing.timestamp = 0
        mav.signing.sign_outgoing = True

    # set the WP_RADIUS parameter on the MAV at the end of the link
    #print(mav.param_set_send(7, 1, "WP_RADIUS", 101, mavlink.MAV_PARAM_TYPE_REAL32))
    
    #print(mav.set_position_target_local_ned_encode(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1))
    # alternatively, produce a MAVLink_param_set object 
    # this can be sent via your own transport if you like
    m = mav.param_set_encode(7, 1, "WP_RADIUS", 101, mavlink.MAV_PARAM_TYPE_REAL32)
    print(type(m))
    print('\n')
    m.pack(mav)
    #print(m.pack(mav))

    # get the encoded message as a buffer
    b = m.get_msgbuf()
    print(type(b))
    print(b)
    print('\n')
    bi=[]
    for c in b:
        print(type(c))
        bi.append(int(c))
    print("Buffer containing the encoded message:")
    print(bi)

    print("Decoding message...")
    # decode an incoming message
    m2 = mav.decode(b)

    # show what fields it has
    print("Got a message with id %u and fields %s" % (m2.get_msgId(), m2.get_fieldnames()))

    # print out the fields
    print(m2)


print("Testing mavlink1\n")
test_protocol(mavlink1)

#print("\nTesting mavlink2\n")
#test_protocol(mavlink2)

#print("\nTesting mavlink2 with signing\n")
#test_protocol(mavlink2, True)
