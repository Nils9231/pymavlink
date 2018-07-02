# Import mavutil
from pymavlink import mavutil

# Create the connection
# From topside computer
master = mavutil.mavlink_connection(
            '/dev/ttyUSB0',
            baud=57600)#mavutil.mavlink_connection('udp:0.0.0.0:14550')



while True:
    msg = master.recv_match()
    if not msg:
        continue
    if msg.get_type() == 'SYS_STATUS':
        print("\n\n*****Got message: %s*****" % msg.get_type())
        print("Message: %s" % msg)
        print("\nAs dictionary: %s" % msg.to_dict())
        # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
        #print("\nSystem status: %s" % msg.system_status)