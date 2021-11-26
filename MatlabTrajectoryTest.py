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
        print("sending thrust:", _desired_thrust)
        #print("sending roll:", _desired_roll)
        #print("sending pitch:", _desired_pitch)
        #print("sending yawrate:", _desired_yawrate)

            
        self._cf.commander.send_setpoint(_desired_roll, _desired_pitch, _desired_yawrate, int(_desired_thrust)) # Send Set points to the drone

class ControlParameters:
    """Parameters used for the PID controller"""
    def __init__(self):
        self.Hz = 10  # Frequency of the code
            
        self.kp = 5643#3741 
        self.ki = 500#1000      # Proportional, Integral and Derivative coefficient for the z-axis
        self.kd = 26000#3000
            
        self.kp_xy = 3
        self.ki_xy = 1        # Proportional, Integral and Derivative coefficient for the x- and y-axis
        self.kd_xy = 4 

        self.kp_yaw = 50#114.58 
        self.ki_yaw =  0.6  # Proportional, Integral and Derivative coefficient for the yaw rotation
        self.kd_yaw =  6 
            
        self.offset_roll    = 0 # Difference between calculated input required to hover compared to actual input required 
        self.offset_pitch   = 0
        self.offset_yawrate = 0
        self.offset_thrust  = 0 

            # Lists containing 4 last errors measured from the z, x, y and yaw positions
        self.error_x_list   = [9999, 9999, 9999, 9999]
        self.error_y_list   = [9999, 9999, 9999, 9999]
        self.error_z_list   = [9999, 9999, 9999, 9999]
        self.error_yaw_list = [9999, 9999, 9999, 9999]

            # Summarized error on x, y, z and yaw
        self.inte_x = 0 
        self.inte_y = 0
        self.inte_z = 0
        self.inte_yaw = 0

        self.trajectory_array = self._getTrajectory()

    def _getTrajectory(self):
        start_point=[0.,0.,0.,0.];
        via_points=([[0.,0.,1.,4.],[0.,1.,1.,8.],[-1.,1.,1.,12.],[-1.,0.,1.,16.],[0.,0.,1.,20.]])
        end_point=[0.,0.,0.,24.]
        eng = matlab.engine.start_matlab()
        path = "C:\\Users\\SebBl\\Documents\\WrittenPrograms\\Matlab" # specify your path
        eng.addpath(path, nargout= 0)
        print("Starting Matlab code")
        return eng.GetTrajectoryxyz(matlab.double(start_point),matlab.double(via_points),matlab.double(end_point),10)


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
    
def PID_Controller(_trajectory_thrust, _statevector, _t):
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
    #rotationvector[2] = rotationvector[2] -0.22
    #print("X position:", statevector[0])
    #print("Y position:", statevector[1])
    print("Z position:", statevector[2])
    #print("Yaw position:", rotationvector[2])

    SetPoint_x   = Control_Param.trajectory_array[0][interals]
    SetPoint_y   = Control_Param.trajectory_array[3][interals]
    SetPoint_z   = Control_Param.trajectory_array[6][interals]
    SetPoint_yaw = 0

    #print("x setpoint is:", SetPoint_x)
    #print("y setpoint is:", SetPoint_y)
    print("z setpoint is:", SetPoint_z)
    #print("yaw setpoint is:", SetPoint_yaw)

    SetPoint_roll    = Control_Param.trajectory_array[9][interals] + Control_Param.offset_roll
    SetPoint_pitch   = Control_Param.trajectory_array[10][interals] + Control_Param.offset_pitch
    SetPoint_yawrate = 0 + Control_Param.offset_yawrate
    SetPoint_thrust  = _trajectory_thrust + Control_Param.offset_thrust

    #print("roll setpoint is:", SetPoint_roll)
    #print("pitch setpoint is:", SetPoint_pitch)
    print("thrust setpoint is:", SetPoint_thrust)
    #print("yawrate setpoint is:", SetPoint_yawrate)

    error_x   = SetPoint_x - _statevector[0]
    error_y   = SetPoint_y - _statevector[1]
    error_z   = SetPoint_z - _statevector[2]
    error_yaw = SetPoint_yaw - rotationvector[2]

    #print("x error is:", error_x)
    #print("y error is:", error_y)
    print("Z error is:", error_z)
    #print("yaw error is:", error_yaw)

    if _statevector[2] is not 4.0:
        Control_Param.inte_x   = Control_Param.integrator(error_x,   Control_Param.inte_x)
        Control_Param.inte_y   = Control_Param.integrator(error_y,   Control_Param.inte_y)
        Control_Param.inte_z   = Control_Param.integrator(error_z,   Control_Param.inte_z)
        Control_Param.inte_yaw = Control_Param.integrator(error_yaw, Control_Param.inte_yaw)

    #print("x integral is:", Control_Param.inte_x)
    #print("y integral is:", Control_Param.inte_y)
    print("Z integral is:", Control_Param.inte_z)
    #print("yaw integral is:", Control_Param.inte_yaw)

    diff_x   = Control_Param.diff(error_x,   Control_Param.error_x_list)
    diff_y   = Control_Param.diff(error_y,   Control_Param.error_y_list)
    diff_z   = Control_Param.diff(error_z,   Control_Param.error_z_list)
    diff_yaw = Control_Param.diff(error_yaw, Control_Param.error_yaw_list)

    #print("x diff is:", diff_x)
    #print("y diff is:", diff_y)
    #print("Z diff is:", diff_z)
    #print("yaw diff is:", diff_yaw)

    ControlVariable_roll    = error_y*Control_Param.kp_xy    + Control_Param.inte_y*Control_Param.ki_xy    + diff_y*Control_Param.kd_xy
    ControlVariable_pitch   = error_x*Control_Param.kp_xy    + Control_Param.inte_x*Control_Param.ki_xy    + diff_x*Control_Param.kd_xy
    ControlVariable_yawrate = error_yaw*Control_Param.kp_yaw + Control_Param.inte_yaw*Control_Param.ki_yaw + diff_yaw*Control_Param.kd_yaw
    ControlVariable_thrust  = error_z*Control_Param.kp       + Control_Param.inte_z*Control_Param.ki       + diff_z*Control_Param.kd

    #print("Control variable of roll is:", ControlVariable_roll)
    #print("Control variable of pitch is:", ControlVariable_pitch)
    #print("Control variable of yawrate is:", ControlVariable_yawrate)
    print("Control variable of thrust is:", ControlVariable_thrust)

    SetPoint_roll    = -(SetPoint_roll    + ControlVariable_roll)
    SetPoint_pitch   =   SetPoint_pitch   + ControlVariable_pitch
    SetPoint_yawrate = -(SetPoint_yawrate + ControlVariable_yawrate)
    SetPoint_thrust  =   SetPoint_thrust  + ControlVariable_thrust

    SetPoint_roll, SetPoint_pitch = Control_Param.convert2baseframe(SetPoint_roll, SetPoint_pitch, rotationvector[2])

    if _t > 1:
        DroneConnector.send_drone_command(SetPoint_roll, SetPoint_pitch, SetPoint_yawrate, SetPoint_thrust)
    else:
        DroneConnector.send_drone_command(0,0,0,0)
    return _statevector


if __name__ == '__main__':
    # Initialize the low-level drivers
    OBJECT="gr562"
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
    #client.ConfigureWireless()
    Control_Param = ControlParameters()
    print(Control_Param.trajectory_array[8])

    cflib.crtp.init_drivers()
    DroneConnector = DroneCommands(uri)
    interals = 0

    stop = False
    stopping_counter = 0
    start_time = time.time()
    print("Starting trajectory sequence")
    for thrust in Control_Param.trajectory_array[11]:
        if interals == len(Control_Param.trajectory_array[11])-1:
            break
        else:
            interals += 1
        if keyboard.is_pressed('space'):
            stop = True
        if stop == False:
            statevector = PID_Controller(thrust, statevector, interals)
        else:
            statevector = PID_Controller(45000, statevector, interals)
            stopping_counter += 1
            if (stopping_counter == 20):
                break
        result = time.time() - start_time
        if (result < 1/Control_Param.Hz):
            time.sleep((1/Control_Param.Hz) - result)
        else:
            print("code too slow")
            print("Frequence was " + str(1/result) + " Hz")
        start_time = time.time()
