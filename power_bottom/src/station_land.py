#!/usr/bin/env python3
import rospy
from clover import srv
from std_srvs.srv import Trigger, TriggerResponse
import math

home_lat = None
home_lon = None
home_alt = None
home_fixed = False

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_led = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)


def get_sat():
    return get_telemetry().satellites

def led_red():
    set_led(r=255, g=0, b=0)

def led_yellow():
    set_led(r=255, g=150, b=0)

def led_green():
    set_led(r=0, g=255, b=0)

def update_led():
    s = get_sat()
    if s <= 5:
        led_red()
    elif s <= 10:
        led_yellow()
    else:
        led_green()

def wait_arrival(tolerance=0.3):
    while not rospy.is_shutdown():
        update_led()
        t = get_telemetry(frame_id='navigate_target')
        if math.sqrt(t.x**2 + t.y**2 + t.z**2) < tolerance:
            break
        rospy.sleep(0.2)

def set_home(req):
    global home_lat, home_lon, home_alt, home_fixed
    t = get_telemetry()
    if math.isnan(t.lat):
        return TriggerResponse(success=False, message="gps_err")
    home_lat, home_lon, home_alt = t.lat, t.lon, t.alt
    home_fixed = True
    return TriggerResponse(success=True, message="ok")

def home_set(req):
    return TriggerResponse(success=home_fixed, message="ok")

def go_home(req):
    if not home_fixed:
        return TriggerResponse(success=False, message="no_home")
    t = get_telemetry()
    navigate_global(lat=home_lat, lon=home_lon, z=t.z, yaw=float('nan'), speed=1)
    wait_arrival()
    return TriggerResponse(success=True, message="ok")

rospy.init_node('station land')

rospy.Service('set_home', Trigger, set_home)
rospy.Service('is_home_set', Trigger, home_set)
rospy.Service('go_home', Trigger, go_home)
rospy.loginfo("station land Node start")

rospy.spin()
