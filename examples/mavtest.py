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

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

s_port = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate = 57600,
    parity = serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

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


s_port.isOpen()

i=0
while i < 255:
    f = fifo()
    mav = mavlink1.MAVLink(f)

    out = read_port()
    time.sleep(0.05)
    if out == []:
        continue


    bi = []
    bi = ([ord(c) for c in out])

    while bi[0]!=254:
        bi[0]=[]
        if bi==[]:
            break
    n=1
    while 1:
        if bi[n]==254:
            bi[n:]=[]
            break


    print('bi_len ' + str(len(bi)))
    #if len(bi) < 10:
    #    continue
    print('bi = ' + str(bi))
    #headerlen = 6
    #print('header' + str(bi[:headerlen]))
    print('\n')
    #print(mav.decode(bi))
    #if bi[0] == '254':
    #    print(mav.decode(bi))

    #"""
    i+=1

s_port.close()




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
    #print(m)
    m.pack(mav)
    #print(m.pack(mav))
    # get the encoded message as a buffer
    b = m.get_msgbuf()
    #print(b)
    bi=[]
    for c in b:
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

print("\nTesting mavlink2\n")
test_protocol(mavlink2)

print("\nTesting mavlink2 with signing\n")
test_protocol(mavlink2, True)
