from sensor_msgs.msg import Range
from aruco_pose.msg import MarkerArray
from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
from clover.srv import SetLEDEffect
import time
import math
import rospy

rospy.init_node('marker_land')

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_altitude = rospy.ServiceProxy('set_altitude', srv.SetAltitude)
set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
set_yaw_rate = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

def markers_callback(msg):
    for marker in msg.markers:
        if marker.id == 5:
            if time.time() - start_time < 10:
                set_position(x=0, y=0, z=1, frame_id='aruco_5', yaw = math.nan)
                print(time.time() - start_time)
            else:
                land_wait()

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

range = 0

def range_callback(msg):
    global range
    range = msg.range

z = 0.75

def land_wait():
    global z
    
    while get_telemetry().armed:
        rospy.sleep(0.2)
        z -= 0.25
        if range >= 0.35:
            set_position(x=0, y=0, z=z, frame_id='aruco_5', yaw = math.nan)
        else:
            land_wait()


start_time = time.time()
rospy.Subscriber('rangefinder/range', Range, range_callback)
aruco_sub = rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)
