#!/usr/bin/env python

######IMPORTS#################
import rospy
from sensor_msgs.msg import Image
import cv2
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from inference_model import *
from object_detection.utils import visualization_utils as viz_utils

### Tensorflow Object Detection Model Load ###
parser = argparse.ArgumentParser()
parser.add_argument('--model_path', '-m', dest='model_path', default=None)

args = parser.parse_args()
model_path = os.path.join(args.model_path, 'saved_model/')
category_index = {1 : {'id' : 1, 'name':'helipad'},
                  2 : {'id' : 2, 'name':'Vehicle'},
                  3 : {'id' : 3, 'name':'Person'},}


### Global Variables - OBD Model ###
detect_fn = load_model(model_path)
min_score_thresh = 0.8

### Global Variables - Vehicle ###
vehicle = connect('udp:127.0.0.1:14551', wait_ready=True) # connect to drone in SITL
vehicle.parameters['PLND_ENABLED'] = 1 # PLND is parameter of ardupilot
vehicle.parameters['PLND_TYPE'] = 1 # companion computer
vehicle.parameters['PLND_EST_TYPE'] = 0 # '0' means 'precision_landing' only use camera sensor
vehicle.parameters['LAND_SPEED'] = 30 # cm/s : 30cm landing per second

velocity = .2 # m/s
takeoff_height = 15 # meters


newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

marker_size = 20 ## cm


horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180) ##62.2 for picam V2, 53.5 for V1
vertical_fov = 48.8 * (math.pi / 180) ##48.8 for V2, 41.41 for V1

found_count = 0
notfound_count = 0


###############CAMERA INTRINSICS################
dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[1061.6538553425996, 0.0, 640.5], [0.0, 1061.6538553425996, 360.5], [0.0, 0.0, 1.0]] # from rostopic camera info
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

#################
time_last = 0
time_to_wait = .01  # 100ms



##############fn for arm and takeoff############################
def arm_and_takeoff(targetHeight):
    while vehicle.is_armable != True:
        print("Waiting for vehicle to become armable.")
        time.sleep(1)
    print("Vehicle is now armable")

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")

    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    print("Look out! Virtual props are spinning!!")

    vehicle.simple_takeoff(targetHeight) ##meters

    while True:
        print("Current Altitude: %d" % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= .95 * targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!!")

    return None

##Send a velocity command with +x being the heading of the drone. fn for velocity for drone
def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_land_message(x,y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0, 0, 0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def msg_receiver(message):
    global notfound_count, found_count, time_last, time_to_wait, id_to_find, detect_fn

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message) # deserialize image data into array
        np_data_with_detections = np_data.copy()
        try:
            if vehicle.location.global_relative_frame.alt <= 0:
                print("Landing Succeed! Disarming..")
                vehicle.armed == False

            else:
                input_tensor = np.expand_dims(np_data, 0)
                detections = detect_fn(input_tensor)

                viz_utils.visualize_boxes_and_labels_on_image_array(
                    np_data_with_detections,
                    detections['detection_boxes'][0].numpy(),
                    detections['detection_classes'][0].numpy().astype(np.int32),
                    detections['detection_scores'][0].numpy(),
                    category_index,
                    use_normalized_coordinates=True,
                    max_boxes_to_draw=200,
                    min_score_thresh=.40,
                    agnostic_mode=False)

                new_msg = rnp.msgify(Image, np_data_with_detections, encoding='rgb8')
                newimg_pub.publish(new_msg)


                best_heli_index = np.where(detections['detection_classes'][0]==1)[0][0]
                best_heli_score = np.take(detections['detection_scores'][0], best_heli_index)

                if best_heli_score >= min_score_thresh:
                    y_min, x_min, y_max, x_max = detections['detection_boxes'][0][best_heli_index].numpy()

                    x_center = (x_min + x_max) / 2
                    y_center = (y_min + y_max) / 2

                    # print(x_center, y_center)
                    if x_center >= 0.5:
                        x_vel = (x_center - 0.5)  # positive direction (east)
                    else:
                        x_vel = (x_center - 0.5)  # negative direction (west)
                    if y_center >= 0.5:
                        y_vel = (0.5 - y_center)  # negative direction (south)
                    else:
                        y_vel = (0.5 - y_center)  # positive direction(north)

                    send_local_ned_velocity(y_vel, x_vel, 0.2)

                else:
                    send_local_ned_velocity(0, 0, 0.1)

                    msg_misc = "Auto Landing.. - Current Altitude %.3f meters" % float(vehicle.location.global_relative_frame.alt)
                    cv2.putText(np_data, msg_misc, (10, 50), 0, .7, (255, 0, 0), thickness=2)
                    new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
                    newimg_pub.publish(new_msg)

        except Exception as e:
            print('Target likely not found')
            print(e)
            notfound_count = notfound_count + 1

    else:
        return None

def subscriber():
    rospy.init_node('drone_node', anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    rospy.spin()

if __name__ == '__main__':
    try:
        arm_and_takeoff(takeoff_height)
        # time.sleep(1)
        # send_local_ned_velocity(velocity,0,0)
        time.sleep(1)
        subscriber()
    except rospy.ROSInterruptException:
        pass
