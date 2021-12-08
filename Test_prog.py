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

import matlab.engine

import keyboard
import math

import numpy

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')

logging.basicConfig(level=logging.ERROR)

class DroneCommands:
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

    def send_drone_command(self, _desired_roll, _desired_pitch, _desired_yawrate, _desired_thrust):
            # Ensures maximum and minimum thrust isn't exceeded
        if (_desired_thrust < 0):
            _desired_thrust = 0
        elif (_desired_thrust > 0xFFFF):
            _desired_thrust = 0xFFFF

        max_Rot = 20 # max/min value of Roll and Pitch
        if (_desired_roll < -max_Rot):
            _desired_roll = -max_Rot
        elif (_desired_roll > max_Rot):
            _desired_roll = max_Rot
        if (_desired_pitch < -max_Rot):
            _desired_pitch = -max_Rot
        elif (_desired_pitch > max_Rot):
            _desired_pitch = max_Rot

            # Terminal outputs of values for troubleshooting
        #print("sending thrust:", _desired_thrust)
        #print("sending roll:", _desired_roll)
        #print("sending pitch:", _desired_pitch)
        #print("sending yawrate:", _desired_yawrate)

            
        self._cf.commander.send_setpoint(_desired_roll, _desired_pitch, _desired_yawrate, int(_desired_thrust)) # Send Set points to the drone

class PID_Parameters:
    def __init__(self, title, k_p, k_i, k_d):
        self.title = title
        self.kp    = k_p
        self.ki    = k_i
        self.kd    = k_d
        self.error_list = [9999, 9999, 9999, 9999]
        self.inte  = 0


class ControlParameters:
    """Parameters used for the PID controller"""
    def __init__(self, statevector):
        self.Hz = 10  # Frequency of the code

        self.Param_x   = PID_Parameters('x',      3.0,   1.0,     7.0) 
        self.Param_y   = PID_Parameters('y',      3.0,   1.0,     7.0)
        self.Param_z   = PID_Parameters('z',   5643.0, 500.0, 26000.0)   
        self.Param_yaw = PID_Parameters('yaw',   50.0,   0.6,     6.0)   

        self.offset_roll    = 0 # Difference between calculated input required to hover compared to actual input required 
        self.offset_pitch   = 0
        self.offset_yawrate = 0
        self.offset_thrust  = 0 

        self._pos_x, self._vel_x, self._acc_x, self._pos_y, self._vel_y, self._acc_y, self._pos_z, self._vel_z, self._acc_z, self._roll, self._pitch, self._thrust = self._getTrajectory(statevector)
        self.x_array      = [self._pos_x, self._vel_x, self._acc_x ]
        self.y_array      = [self._pos_y, self._vel_y, self._acc_y ]
        self.z_array      = [self._pos_z, self._vel_z, self._acc_z ]
        self.inputs_array = [self._roll,  self._pitch, self._thrust]
        

    def _getTrajectory(self, statevector):
        #start_point=statevector
        #start_point.append(0)
        start_point=[ 0.,0.,0.,0.]
        via_points = ([[ 0.,0.,1., 5.],[ -1.,0.,1.,10.],[-1.,1.,1.,15.],[0.,1.,1.,20.],[ 0.,0.,1.,25.]])
        end_point = [0.,0.,0.,30.]
        print("Starting Matlab engine")
        eng = matlab.engine.start_matlab()
        path = "C:\\Users\\SebBl\\Documents\\WrittenPrograms\\Matlab" # specify your path
        eng.addpath(path, nargout= 0)
        print("Starting Matlab code")
        return eng.GetTrajectoryxyz(matlab.double(start_point),matlab.double(via_points),matlab.double(end_point),matlab.double([self.Hz]))


    def integrator(self, _current_error, _inte):
        _inte = (1/self.Hz)*_current_error + _inte
        return _inte


    def diff(self, _error, error_list):
        error_list.append(_error)
        error_list.pop(0)
        if error_list[0] is not 9999:
            a = (error_list[len(error_list)-1] - error_list[len(error_list)-3])/((1/Control_Param.Hz)*2)
            b = (error_list[len(error_list)-2] - error_list[len(error_list)-4])/((1/Control_Param.Hz)*2)
        else:
            a = 0
            b = 0 
        _result = (a+b)/2
        return _result

    def convert2baseframe(self, _roll, _pitch, _yaw):
        roll  = (math.cos(_yaw)*_roll  + math.sin(_yaw)*_pitch) 
        pitch = (math.cos(_yaw)*_pitch - math.sin(_yaw)*_roll)  
        return roll, pitch
    

def PID_Controller(_current_Position, _desired_Position, _PID_Param):
    error = _desired_Position - _current_Position

    if _current_Position is not 0.0 or _current_Position is not 4.0:
        _PID_Param.inte = Control_Param.integrator(error, _PID_Param.inte)

    diff = Control_Param.diff(error, _PID_Param.error_list)

    ControlVariable = error*_PID_Param.kp + _PID_Param.inte*_PID_Param.ki    + diff*_PID_Param.kd

    return ControlVariable


def Controller(_statevector, _t):
    try:
        client.GetFrame()    
        list,occ = client.GetSegmentGlobalTranslation(OBJECT, OBJECT )
        _statevector = [float(list[0])/1000,float(list[1])/1000,float(list[2])/1000]
        list_rpy,occ = client.GetSegmentGlobalRotationEulerXYZ(OBJECT, OBJECT )
        rotationvector = [float(list_rpy[0]),float(list_rpy[1]),float(list_rpy[2])]
    except ViconDataStream.DataStreamException as e:
        print(e)
    if _statevector[2] == 0.0:
        _statevector[2] = 4.0

    SetPoint_roll    = -(Control_Param.inputs_array[0][interals] + PID_Controller(_statevector[1], Control_Param.y_array[0][interals], Control_Param.Param_y))
    SetPoint_pitch   = -(Control_Param.inputs_array[1][interals] - PID_Controller(_statevector[0], Control_Param.x_array[0][interals], Control_Param.Param_x))
    SetPoint_thrust  =   Control_Param.inputs_array[2][interals] + PID_Controller(_statevector[2], Control_Param.z_array[0][interals], Control_Param.Param_z)
    SetPoint_yawrate = -(                                      0 + PID_Controller(rotationvector[2],                                0, Control_Param.Param_yaw))

    SetPoint_roll, SetPoint_pitch = Control_Param.convert2baseframe(SetPoint_roll, SetPoint_pitch, rotationvector[2])

    if _t > 1:
        DroneConnector.send_drone_command(SetPoint_roll, SetPoint_pitch, SetPoint_yawrate, SetPoint_thrust)
    else:
        DroneConnector.send_drone_command(0,0,0,0)
    return _statevector


if __name__ == '__main__':
    OBJECT="gr562"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('host', nargs='?', help="Host name, in the format of server:port", default = "192.168.1.33:801")  #set ip adressen. 
    args = parser.parse_args()
    client = ViconDataStream.Client()
    client.Connect( args.host )
    client.SetBufferSize( 1 )
    client.EnableSegmentData()

    HasFrame = False
    print("Searching for frame.....")
    while not HasFrame:
        try:
            client.GetFrame()
            list,occ= client.GetSegmentGlobalTranslation(OBJECT, OBJECT )
            statevector=[float(list[0])/1000,float(list[1])/1000,float(list[2])/1000]

            list_rpy,occ= client.GetSegmentGlobalRotationEulerXYZ(OBJECT, OBJECT )
            rotationvector=[float(list[0]),float(list[1]),float(list[2])]

            HasFrame = True
        except ViconDataStream.DataStreamException as e:
            client.GetFrame()
    client.SetAxisMapping( ViconDataStream.Client.AxisMapping.EForward, ViconDataStream.Client.AxisMapping.ELeft, ViconDataStream.Client.AxisMapping.EUp )

    Control_Param = ControlParameters(statevector)

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    DroneConnector = DroneCommands(uri)
    interals = 0

    print("Starting trajectory sequence")
    for thrust in Control_Param.inputs_array[2]:
        start_time = time.time()
        statevector = Controller(statevector, interals)
        print(statevector[0], ',', statevector[1], ',', statevector[2])
        interals += 1

        result = time.time() - start_time
        if (result < 1/Control_Param.Hz):
            time.sleep((1/Control_Param.Hz) - result)
        else:
            print("code too slow")
            print("Frequence was " + str(1/result) + " Hz")
    
    DroneConnector.send_drone_command(0,0,0,0)


