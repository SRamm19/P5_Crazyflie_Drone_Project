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

import keyboard
import math

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

    def control(self, _state_vector, _desired_Position):#, _kp):
        #if _state_vector[2] == 0:
        #    _state_vector[2] = 2.0
        _difference = _desired_Position - _state_vector
        print("Position difference is: " + str(_difference))
        #_input_change = _difference * _kp
        return _difference

    def send_drone_command(self, _desired_roll, _desired_pitch, _desired_yawrate, _desired_thrust):
        #print("sending thrust: " + str(_desired_thrust))

        if (_desired_thrust < 0):
            _desired_thrust = 10000
        elif (_desired_thrust > 0xFFFF):
            _desired_thrust = 0xFFFF
        if (_desired_roll < -15):
            _desired_roll = -15
        elif (_desired_roll > 15):
            _desired_roll = 15
        if (_desired_pitch < -15):
            _desired_pitch = -15
        elif (_desired_pitch > 15):
            _desired_pitch = 15
        print("sending thrust: ", _desired_thrust)
        print("sending roll: ", _desired_roll)
        print("sending pitch: ", _desired_pitch)
        self._cf.commander.send_setpoint(_desired_roll, _desired_pitch, _desired_yawrate, int(_desired_thrust))

class ControlParameters:
    """Parameters used for the PID controller"""
    def __init__(self):
        self.Hz = 10

        self.kp = 3741
        self.kd = 5
        self.ki = 10

        self.kp_xy = 3
        self.kd_xy = 3#0.025
        self.ki_xy = 0#0.1

        self.kp_yaw = 2
        self.kd_yaw = 1
        self.ki_yaw = 0

        self.offset = -1000

        self.pose_z_list = [9999, 9999, 9999, 9999]
        self.pose_yaw_list = [9999, 9999, 9999, 9999]
        self.pose_x_list = [9999, 9999, 9999, 9999]
        self.pose_y_list = [9999, 9999, 9999, 9999]

        self.inte_y = 0
        self.inte_x = 0
        self.inte_z = 0
        self.inte_yaw = 0

    def integrator(self, _i, _current_error, _inte):
        _inte = (_i/controls.Hz)*_current_error + _inte
        return _inte

    def diff(self, _y, pose_list):
        pose_list.append(_y)
        pose_list.pop(0)
        if pose_list[0] is not 9999:
            a = (pose_list[len(pose_list)-1] - pose_list[len(pose_list)-3])/((1/controls.Hz)*2)
            b = (pose_list[len(pose_list)-2] - pose_list[len(pose_list)-4])/((1/controls.Hz)*2)
        else:
            a = 0
            b = 0 
        _result = (a+b)/2
        return _result



def do_Stuff(_trajectory_thrust, _statevector, _t):
    try:
        client.GetFrame()    
        list,occ= client.GetSegmentGlobalTranslation(OBJECT, OBJECT )
        _statevector=(float(list[0])/1000,float(list[1])/1000,float(list[2])/1000)

        list_rpy,occ= client.GetSegmentGlobalRotationEulerXYZ(OBJECT, OBJECT )
        rotationvector=(float(list_rpy[0])*180/3.141592,float(list_rpy[1])*180/3.141592,float(list_rpy[2])*180/3.141592)

    except ViconDataStream.DataStreamException as e:
        print(e)

    print("Z position:" + str(statevector[2]))

    yaw_change = rotationvector[2]*controls.kp_yaw
    hover_thrust_change = le.control(_statevector[2], trajectory_position[interals])
    controls.inte_x = controls.integrator(_t, 0 - _statevector[0], controls.inte_x)
    controls.inte_y = controls.integrator(_t, 0 - _statevector[1], controls.inte_y)
    controls.inte_z = controls.integrator(_t, trajectory_position[interals] - _statevector[2], controls.inte_z)
    controls.inte_yaw = controls.integrator(_t, 0 - rotationvector[2], controls.inte_yaw)

    if (rotationvector[2] > -5 and rotationvector[2] < 5):
        roll_change  =  -le.control(_statevector[1], 0)
        pitch_change = le.control(_statevector[0], 0)
        x_diff = controls.diff(pitch_change, controls.pose_x_list)
        y_diff = controls.diff(roll_change, controls.pose_y_list)
        DoStuff_inte_x = controls.inte_x
        DoStuff_inte_y = controls.inte_x
    else:
        #pass
        roll_change  = 0
        pitch_change = 0
        x_diff = 0
        y_diff = 0
        DoStuff_inte_x = 0
        DoStuff_inte_y = 0 
    print("x diff is: ", x_diff*controls.kd_xy)
    print("y diff is: ", y_diff*controls.kd_xy)


    le.send_drone_command(0 + roll_change*controls.kp_xy + y_diff*controls.kd_xy + DoStuff_inte_y*controls.ki_xy, 
                          0 + pitch_change*controls.kp_xy + x_diff*controls.kd_xy + DoStuff_inte_x*controls.ki_xy, 
                          0 + yaw_change*controls.kp_yaw + controls.inte_yaw*controls.ki_yaw,
                          _trajectory_thrust + hover_thrust_change*controls.kp + controls.offset + controls.diff(hover_thrust_change, controls.pose_z_list)*controls.kd + controls.inte_z*controls.ki)
    return _statevector




trajectory_thrust = [55578, 55324, 55071, 54817, 54564, 54310, 54056, 53803, 53549, 53295,
                     53042, 52788, 52535, 52281, 52027, 51774, 51520, 51266, 51013, 50759, 
                     50506, 50252, 49998, 49745, 49491, 49237, 48984, 48730, 48477, 48223, 
                     47969, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774]
trajectory_position = [ 0.0,    0.0033, 0.0127, 0.028,  0.0486, 0.0741, 0.104,  0.1379, 0.1754, 0.216, 
                        0.2593, 0.3047, 0.352,  0.4006, 0.4501, 0.5,    0.5499, 0.5994, 0.648,  0.6953, 
                        0.7407, 0.784,  0.8246, 0.8621, 0.896,  0.9259, 0.9514, 0.972,  0.9873, 0.9967, 
                        1.0,    1.0,    1.0,    1.0,    1.0,    1.0,    1.0,    1.0,    1.0,    1.0,    1.0]

#trajectory_thrust = [ 51793, 51793, 51793, 51793, 51792, 51792, 51792, 51792, 51792, 51792, 51792, 51792, 51792, 51792, 51792, 51792, 51792, 51792, 51792, 51792, 51791, 51791, 51791, 51791, 51791, 51791, 51791, 51791, 51791, 51791, 51791, 51791, 51791, 51791, 51791, 51790, 51790, 51790, 51790, 51790, 51790, 51790, 51790, 51790, 51790, 51790, 51790, 51790, 51790, 51790, 51790, 51789, 51789, 51789, 51789, 51789, 51789, 51789, 51789, 51789, 51789, 51789, 51789, 51789, 51789, 51789, 51789, 51788, 51788, 51788, 51788, 51788, 51788, 51788, 51788, 51788, 51788, 51788, 51788, 51788, 51788, 51788, 51788, 51787, 51787, 51787, 51787, 51787, 51787, 51787, 51787, 51787, 51787, 51787, 51787, 51787, 51787, 51787, 51787, 51786, 51786, 51786, 51786, 51786, 51786, 51786, 51786, 51786, 51786, 51786, 51786, 51786, 51786, 51786, 51785, 51785, 51785, 51785, 51785, 51785, 51785, 51785, 51785, 51785, 51785, 51785, 51785, 51785, 51785, 51785, 51784, 51784, 51784, 51784, 51784, 51784, 51784, 51784, 51784, 51784, 51784, 51784, 51784, 51784, 51784, 51784, 51783, 51783, 51783, 51783, 51783, 51783, 51783, 51783, 51783, 51783, 51783, 51783, 51783, 51783, 51783, 51783, 51782, 51782, 51782, 51782, 51782, 51782, 51782, 51782, 51782, 51782, 51782, 51782, 51782, 51782, 51782, 51781, 51781, 51781, 51781, 51781, 51781, 51781, 51781, 51781, 51781, 51781, 51781, 51781, 51781, 51781, 51781, 51780, 51780, 51780, 51780, 51780, 51780, 51780, 51780, 51780, 51780, 51780, 51780, 51780, 51780, 51780, 51780, 51779, 51779, 51779, 51779, 51779, 51779, 51779, 51779, 51779, 51779, 51779, 51779, 51779, 51779, 51779, 51779, 51778, 51778, 51778, 51778, 51778, 51778, 51778, 51778, 51778, 51778, 51778, 51778, 51778, 51778, 51778, 51777, 51777, 51777, 51777, 51777, 51777, 51777, 51777, 51777, 51777, 51777, 51777, 51777, 51777, 51777, 51777, 51776, 51776, 51776, 51776, 51776, 51776, 51776, 51776, 51776, 51776, 51776, 51776, 51776, 51776, 51776, 51776, 51775, 51775, 51775, 51775, 51775, 51775, 51775, 51775, 51775, 51775, 51775, 51775, 51775, 51775, 51775, 51775, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51773, 51773, 51773, 51773, 51773, 51773, 51773, 51773, 51773, 51773, 51773, 51773, 51773, 51773, 51773, 51772, 51772, 51772, 51772, 51772, 51772, 51772, 51772, 51772, 51772, 51772, 51772, 51772, 51772, 51772, 51772, 51771, 51771, 51771, 51771, 51771, 51771, 51771, 51771, 51771, 51771, 51771, 51771, 51771, 51771, 51771, 51771, 51770, 51770, 51770, 51770, 51770, 51770, 51770, 51770, 51770, 51770, 51770, 51770, 51770, 51770, 51770, 51770, 51769, 51769, 51769, 51769, 51769, 51769, 51769, 51769, 51769, 51769, 51769, 51769, 51769, 51769, 51769, 51768, 51768, 51768, 51768, 51768, 51768, 51768, 51768, 51768, 51768, 51768, 51768, 51768, 51768, 51768, 51768, 51767, 51767, 51767, 51767, 51767, 51767, 51767, 51767, 51767, 51767, 51767, 51767, 51767, 51767, 51767, 51767, 51766, 51766, 51766, 51766, 51766, 51766, 51766, 51766, 51766, 51766, 51766, 51766, 51766, 51766, 51766, 51766, 51765, 51765, 51765, 51765, 51765, 51765, 51765, 51765, 51765, 51765, 51765, 51765, 51765, 51765, 51765, 51764, 51764, 51764, 51764, 51764, 51764, 51764, 51764, 51764, 51764, 51764, 51764, 51764, 51764, 51764, 51764, 51763, 51763, 51763, 51763, 51763, 51763, 51763, 51763, 51763, 51763, 51763, 51763, 51763, 51763, 51763, 51763, 51762, 51762, 51762, 51762, 51762, 51762, 51762, 51762, 51762, 51762, 51762, 51762, 51762, 51762, 51762, 51762, 51761, 51761, 51761, 51761, 51761, 51761, 51761, 51761, 51761, 51761, 51761, 51761, 51761, 51761, 51761, 51761, 51760, 51760, 51760, 51760, 51760, 51760, 51760, 51760, 51760, 51760, 51760, 51760, 51760, 51760, 51760, 51759, 51759, 51759, 51759, 51759, 51759, 51759, 51759, 51759, 51759, 51759, 51759, 51759, 51759, 51759, 51759, 51758, 51758, 51758, 51758, 51758, 51758, 51758, 51758, 51758, 51758, 51758, 51758, 51758, 51758, 51758, 51758, 51757, 51757, 51757, 51757, 51757, 51757, 51757, 51757, 51757, 51757, 51757, 51757, 51757, 51757, 51757, 51757, 51756, 51756, 51756, 51756, 51756, 51756, 51756, 51756, 51756, 51756, 51756, 51756, 51756, 51756, 51756, 51755, 51755, 51755, 51755, 51755, 51755, 51755, 51755, 51755, 51755, 51755, 51755, 51755, 51755, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774, 51774]
#trajectory_position = [ 0, 0.0000166481, 0.0000665185, 0.0001495, 0.000265481, 0.000414352, 0.000596, 0.000810315, 0.00105719, 0.0013365, 0.00164815, 0.00199202, 0.002368, 0.00277598, 0.00321585, 0.0036875, 0.00419081, 0.00472569, 0.005292, 0.00588965, 0.00651852, 0.0071785, 0.00786948, 0.00859135, 0.009344, 0.0101273, 0.0109412, 0.0117855, 0.0126601, 0.013565, 0.0145, 0.015465, 0.0164599, 0.0174845, 0.0185388, 0.0196227, 0.020736, 0.0218786, 0.0230505, 0.0242515, 0.0254815, 0.0267404, 0.028028, 0.0293443, 0.0306892, 0.0320625, 0.0334641, 0.034894, 0.036352, 0.037838, 0.0393519, 0.0408935, 0.0424628, 0.0440597, 0.045684, 0.0473356, 0.0490145, 0.0507205, 0.0524535, 0.0542134, 0.056, 0.0578133, 0.0596532, 0.0615195, 0.0634121, 0.065331, 0.067276, 0.069247, 0.0712439, 0.0732665, 0.0753148, 0.0773887, 0.079488, 0.0816126, 0.0837625, 0.0859375, 0.0881375, 0.0903624, 0.092612, 0.0948863, 0.0971852, 0.0995085, 0.101856, 0.104228, 0.106624, 0.109044, 0.111488, 0.113955, 0.116447, 0.118962, 0.1215, 0.124062, 0.126647, 0.129254, 0.131885, 0.134539, 0.137216, 0.139915, 0.142637, 0.145381, 0.148148, 0.150937, 0.153748, 0.156581, 0.159436, 0.162312, 0.165211, 0.168131, 0.171072, 0.174035, 0.177019, 0.180024, 0.183049, 0.186096, 0.189164, 0.192252, 0.195361, 0.19849, 0.20164, 0.20481, 0.208, 0.21121, 0.21444, 0.217689, 0.220959, 0.224248, 0.227556, 0.230884, 0.234231, 0.237596, 0.240981, 0.244385, 0.247808, 0.251249, 0.254709, 0.258187, 0.261684, 0.265199, 0.268732, 0.272283, 0.275852, 0.279438, 0.283043, 0.286665, 0.290304, 0.293961, 0.297635, 0.301325, 0.305033, 0.308758, 0.3125, 0.316258, 0.320033, 0.323825, 0.327632, 0.331456, 0.335296, 0.339152, 0.343024, 0.346911, 0.350815, 0.354734, 0.358668, 0.362618, 0.366583, 0.370562, 0.374557, 0.378567, 0.382592, 0.386631, 0.390685, 0.394754, 0.398836, 0.402933, 0.407044, 0.411169, 0.415308, 0.41946, 0.423627, 0.427807, 0.432, 0.436207, 0.440427, 0.44466, 0.448905, 0.453164, 0.457436, 0.46172, 0.466017, 0.470326, 0.474648, 0.478982, 0.483328, 0.487686, 0.492056, 0.496437, 0.500831, 0.505236, 0.509652, 0.51408, 0.518519, 0.522969, 0.527429, 0.531901, 0.536384, 0.540877, 0.545381, 0.549896, 0.55442, 0.558955, 0.5635, 0.568055, 0.57262, 0.577195, 0.581779, 0.586373, 0.590976, 0.595589, 0.600211, 0.604842, 0.609481, 0.61413, 0.618788, 0.623454, 0.628129, 0.632812, 0.637504, 0.642204, 0.646912, 0.651628, 0.656352, 0.661084, 0.665823, 0.67057, 0.675324, 0.680086, 0.684855, 0.689631, 0.694413, 0.699203, 0.704, 0.708803, 0.713613, 0.718429, 0.723252, 0.728081, 0.732916, 0.737757, 0.742604, 0.747456, 0.752315, 0.757179, 0.762048, 0.766923, 0.771803, 0.776687, 0.781577, 0.786472, 0.791372, 0.796276, 0.801185, 0.806099, 0.811016, 0.815938, 0.820864, 0.825794, 0.830728, 0.835666, 0.840607, 0.845552, 0.8505, 0.855452, 0.860407, 0.865364, 0.870325, 0.875289, 0.880256, 0.885225, 0.890197, 0.895172, 0.900148, 0.905127, 0.910108, 0.915091, 0.920076, 0.925062, 0.930051, 0.935041, 0.940032, 0.945025, 0.950019, 0.955014, 0.960009, 0.965006, 0.970004, 0.975002, 0.980001, 0.985, 0.99, 0.995, 1.0, 1.005, 1.01, 1.015, 1.02, 1.025, 1.03, 1.03499, 1.03999, 1.04499, 1.04998, 1.05498, 1.05997, 1.06496, 1.06995, 1.07494, 1.07992, 1.08491, 1.08989, 1.09487, 1.09985, 1.10483, 1.1098, 1.11477, 1.11974, 1.12471, 1.12967, 1.13464, 1.13959, 1.14455, 1.1495, 1.15445, 1.15939, 1.16433, 1.16927, 1.17421, 1.17914, 1.18406, 1.18898, 1.1939, 1.19881, 1.20372, 1.20863, 1.21353, 1.21842, 1.22331, 1.2282, 1.23308, 1.23795, 1.24282, 1.24769, 1.25254, 1.2574, 1.26224, 1.26708, 1.27192, 1.27675, 1.28157, 1.28639, 1.2912, 1.296, 1.3008, 1.30559, 1.31037, 1.31515, 1.31991, 1.32468, 1.32943, 1.33418, 1.33892, 1.34365, 1.34837, 1.35309, 1.3578, 1.3625, 1.36719, 1.37187, 1.37655, 1.38121, 1.38587, 1.39052, 1.39516, 1.39979, 1.40441, 1.40902, 1.41363, 1.41822, 1.42281, 1.42738, 1.43195, 1.4365, 1.44104, 1.44558, 1.4501, 1.45462, 1.45912, 1.46362, 1.4681, 1.47257, 1.47703, 1.48148, 1.48592, 1.49035, 1.49476, 1.49917, 1.50356, 1.50794, 1.51231, 1.51667, 1.52102, 1.52535, 1.52967, 1.53398, 1.53828, 1.54256, 1.54684, 1.55109, 1.55534, 1.55957, 1.56379, 1.568, 1.57219, 1.57637, 1.58054, 1.58469, 1.58883, 1.59296, 1.59707, 1.60116, 1.60525, 1.60931, 1.61337, 1.61741, 1.62143, 1.62544, 1.62944, 1.63342, 1.63738, 1.64133, 1.64527, 1.64919, 1.65309, 1.65698, 1.66085, 1.6647, 1.66854, 1.67237, 1.67618, 1.67997, 1.68374, 1.6875, 1.69124, 1.69497, 1.69867, 1.70237, 1.70604, 1.7097, 1.71334, 1.71696, 1.72056, 1.72415, 1.72772, 1.73127, 1.7348, 1.73832, 1.74181, 1.74529, 1.74875, 1.75219, 1.75561, 1.75902, 1.7624, 1.76577, 1.76912, 1.77244, 1.77575, 1.77904, 1.78231, 1.78556, 1.78879, 1.792, 1.79519, 1.79836, 1.80151, 1.80464, 1.80775, 1.81084, 1.8139, 1.81695, 1.81998, 1.82298, 1.82597, 1.82893, 1.83187, 1.83479, 1.83769, 1.84056, 1.84342, 1.84625, 1.84906, 1.85185, 1.85462, 1.85736, 1.86008, 1.86278, 1.86546, 1.86811, 1.87075, 1.87335, 1.87594, 1.8785, 1.88104, 1.88355, 1.88604, 1.88851, 1.89096, 1.89338, 1.89577, 1.89814, 1.90049, 1.90281, 1.90511, 1.90739, 1.90964, 1.91186, 1.91406, 1.91624, 1.91839, 1.92051, 1.92261, 1.92469, 1.92673, 1.92876, 1.93075, 1.93272, 1.93467, 1.93659, 1.93848, 1.94035, 1.94219, 1.944, 1.94579, 1.94755, 1.94928, 1.95099, 1.95266, 1.95432, 1.95594, 1.95754, 1.95911, 1.96065, 1.96216, 1.96365, 1.96511, 1.96654, 1.96794, 1.96931, 1.97066, 1.97197, 1.97326, 1.97452, 1.97575, 1.97695, 1.97812, 1.97926, 1.98038, 1.98146, 1.98252, 1.98354, 1.98454, 1.9855, 1.98643, 1.98734, 1.98821, 1.98906, 1.98987, 1.99066, 1.99141, 1.99213, 1.99282, 1.99348, 1.99411, 1.99471, 1.99527, 1.99581, 1.99631, 1.99678, 1.99722, 1.99763, 1.99801, 1.99835, 1.99866, 1.99894, 1.99919, 1.9994, 1.99959, 1.99973, 1.99985, 1.99993, 1.99998, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]


if __name__ == '__main__':
    # Initialize the low-level drivers

    controls = ControlParameters()
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
            statevector=(float(list[0])/1000,float(list[1])/1000,float(list[2])/1000)

            list_rpy,occ= client.GetSegmentGlobalRotationEulerXYZ(OBJECT, OBJECT )
            rotationvector=(float(list[0])*180/3.141592,float(list[1])*180/3.141592,float(list[2])*180/3.141592)

            HasFrame = True
        except ViconDataStream.DataStreamException as e:
            client.GetFrame()
    client.SetAxisMapping( ViconDataStream.Client.AxisMapping.EForward, ViconDataStream.Client.AxisMapping.ELeft, ViconDataStream.Client.AxisMapping.EUp )
    #client.ConfigureWireless()

    interals = 0
    t_decisec = 0
    cflib.crtp.init_drivers()
    le = MotorRampExample(uri)

    stop = False
    stopping_counter = 0
    start_time = time.time()
    print("Starting trajectory sequence")
    for thrust in trajectory_thrust:
        if interals == len(trajectory_thrust)-1:
            break
        else:
            interals += 1
            t_decisec=interals
        if keyboard.is_pressed('space'):
            stop = True
        if stop == False:
            statevector = do_Stuff(thrust, statevector, t_decisec)

        else:
            statevector = do_Stuff(40000, statevector, t_decisec)
            stopping_counter += 1
            if (stopping_counter == 20):
                break
        result = time.time() - start_time
        if (result < 1/controls.Hz):
            time.sleep((1/controls.Hz) - result)
        else:
            print("code too slow")
            print("Frequence was " + str(1/result) + " Hz")
        start_time = time.time()


    print("Starting hover sequence")
    while True:
        if keyboard.is_pressed('space'):
            stop = True
        if stop == False:
            statevector = do_Stuff(trajectory_thrust[interals], statevector, t_decisec)
        else:
            statevector = do_Stuff(40000, statevector, t_decisec)
            stopping_counter += 1
            if (stopping_counter == 20):
                break
        t_decisec += 1
        result = time.time() - start_time
        if (result < 1/controls.Hz):
            time.sleep((1/controls.Hz) - result)
        else:
            print("code too slow")
            print("Frequence was " + str(1/result) + " Hz")
        start_time = time.time()