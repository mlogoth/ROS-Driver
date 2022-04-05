#!/usr/bin/env python
import sys
import os
import rospy
from roboteq_motor_controller_driver.srv import *

# Servers and handlers for the acceleration, deceleration, maxrpm and ki services
def accel_handler(req):
    print("Calling the command_service to set the acceleration rate for motor 1...")
    command_service_client('AC', 1, req.value*10)
    command_service_client('AC', 2, req.value*10)
    return srv_wrapperResponse('!' + 'AC 1/2 '+ str(req.value))


def decel_handler(req):
    command_service_client('DC', 1, req.value*10)
    command_service_client('DC', 2, req.value*10)
    return srv_wrapperResponse('!' + 'DC 1/2 ' + str(req.value))


def maxrpm_handler(req):
    print("Calling the config_service to set the maximum RPM for motor 1...")
    config_service_client('MXRPM', 1, req.value)
    print("Calling the config_service to set the maximum RPM for motor 2...")
    config_service_client('MXRPM', 2, req.value)
    
    return srv_wrapperResponse('^' + 'MXRPM 1/2 ' + str(req.value))


def ki_handler(req):
    config_service_client('KIG', 1, req.value*1000000)
    config_service_client('KIG', 2, req.value*1000000)
    return srv_wrapperResponse('^' + 'KIG 1/2 ' + str(req.value))


def service_wrapper():
    rospy.init_node('roboteq_service_wrapper_node')
    print('Starting service wrapper node...')
    acc_srv = rospy.Service('set_accel', srv_wrapper, accel_handler)
    decel_srv = rospy.Service('set_decel', srv_wrapper, decel_handler)
    maxrpm_srv = rospy.Service('set_maxrpm', srv_wrapper, maxrpm_handler)
    ki_srv = rospy.Service('set_ki', srv_wrapper, ki_handler)
    print('''
    Set up set_accel, set decel, set_maxrpm and set_ki services. Call them by just defining a value that will be applied to both motors.
    Values do not have to account for roboteq service format. For example if you want to set a 2000 RPM ramp for acceleration,
    or an integral gain of 1.5, just use that value without multipliers.''')
    rospy.spin()


def config_service_client(type, chan, val):
    rospy.wait_for_service('config_service')
    try:
        config_client = rospy.ServiceProxy('config_service', config_srv)
        conf_string_res = config_client(type, chan, val)
        return conf_string_res.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def command_service_client(type, chan, val):
    rospy.wait_for_service('command_service')
    try:
        command_client = rospy.ServiceProxy('command_service', command_srv)
        comm_string_res = command_client(type, chan, val)
        return comm_string_res.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def maintenance_service_client(type, chan, val):
    rospy.wait_for_service('maintenance_service')
    try:
        maintentance_client = rospy.ServiceProxy('maintenance_service', maintenance_srv)
        maint_string_res = maintentance_client(type, chan, val)
        return maint_string_res.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)




if __name__ == "__main__":
    service_wrapper()