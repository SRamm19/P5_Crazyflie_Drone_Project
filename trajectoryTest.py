from __future__ import print_function
from vicon_dssdk import ViconDataStream
import argparse
import time

import logging
import time
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

logging.basicConfig(level=logging.ERROR)

class MotorRampExample:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        
        self._cf.open_link(link_uri)

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._ramp_motors).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def _ramp_motors(self):
        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

    def control(self, _state_vector, _desired_Position):
        #if _state_vector[2] == 0:
        #    _state_vector[2] = 2.0
        _difference = _desired_Position - _state_vector[2]
        print(_difference)
        hover_thrust_change = _difference * kp
        return hover_thrust_change

    def send_drone_command(self, _desired_roll, _desired_pitch, _desired_yawrate, _desired_thrust):
        print("sending thrust: " + str(_desired_thrust))
        if (_desired_thrust >= 0 and _desired_thrust <= 0xFFFF):
            self._cf.commander.send_setpoint(int(_desired_roll), int(_desired_pitch), int(_desired_yawrate), int(_desired_thrust))
        elif (_desired_thrust < 0):
            self._cf.commander.send_setpoint(_desired_roll, _desired_pitch, _desired_yawrate, 1000)
        elif (_desired_thrust > 0xFFFF):
            self._cf.commander.send_setpoint(_desired_roll, _desired_pitch, _desired_yawrate, 0xFFFF)




trajectory = [ 55578, 55324, 55071, 54817, 54564, 54310, 54056, 53803, 53549, 53295,
             53042, 52788, 52535, 52281, 52027, 51774, 51520, 51266, 51013, 50759, 
             50506, 50252, 49998, 49745, 49491, 49237, 48984, 48730, 48477, 48223, 
             47969, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774]

trajectory_position = [ 0.0, 0.0033, 0.0127, 0.028, 0.0486, 0.0741, 0.104, 0.1379, 0.1754, 0.216, 
                       0.2593, 0.3047, 0.352, 0.4006, 0.4501, 0.5, 0.5499, 0.5994, 0.648, 0.6953, 
                       0.7407, 0.784, 0.8246, 0.8621, 0.896, 0.9259, 0.9514, 0.972, 0.9873, 0.9967, 
                       1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]


if __name__ == '__main__':
    # Initialize the low-level drivers
    kp = 5000
    desired_position = [0,0,1]
    offset = 0#-1100


    OBJECT="testing"

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('host', nargs='?', help="Host name, in the format of server:port", default = "192.168.1.34:801")  #set ip adressen. 
    args = parser.parse_args()
    client = ViconDataStream.Client()
    if client.IsConnected():
        print("Error-no connection to the server. ")
    client.Connect( args.host )
    client.SetBufferSize( 1 )
    client.EnableSegmentData()
    HasFrame = False
    while not HasFrame:
        try:
            client.GetFrame()
            HasFrame = True
        except ViconDataStream.DataStreamException as e:
            client.GetFrame()
    client.GetFrame()
    client.SetAxisMapping( ViconDataStream.Client.AxisMapping.EForward, ViconDataStream.Client.AxisMapping.ELeft, ViconDataStream.Client.AxisMapping.EUp )
    client.ConfigureWireless()
    interals = 40#0

    cflib.crtp.init_drivers()

    le = MotorRampExample(uri)

    print("Starting trajectory sequence")
#    for thrust in trajectory:
#        start = time.time()
#        if interals == len(trajectory)-1:
#            break
#        else:
#            interals += 1
#        segmentChildren = client.GetSegmentChildren(OBJECT, OBJECT )     
#        list,occ= client.GetSegmentGlobalTranslation(OBJECT, OBJECT )
#        statevector=(float(list[0])/1000,float(list[1])/1000,float(list[2])/1000)
#        print(statevector[2])
#        hover_thrust_change = le.control(statevector, trajectory_position[interals])
#        le.send_drone_command(0,0,0, thrust + hover_thrust_change + offset)
#        result = time.time() - start
#        if (result < 0.1):
#            time.sleep(0.1 - result)
#        else:
#            print("code too slow")
#        client.GetFrame()
#    print("Starting hover sequence")
    while True:
        start = time.time()
        segmentChildren = client.GetSegmentChildren(OBJECT, OBJECT )     
        list,occ= client.GetSegmentGlobalTranslation(OBJECT, OBJECT )
        statevector=(float(list[0])/1000,float(list[1])/1000,float(list[2])/1000)
        print(trajectory_position[interals])
        hover_thrust_change = le.control(statevector,trajectory_position[interals])
        le.send_drone_command(0,0,0, trajectory[interals] + hover_thrust_change + offset)
        result = time.time() - start
        #if (result < 0.01):
        #    time.sleep(0.01 - result)
        #else:
        #    print("code too slow")
        client.GetFrame()

