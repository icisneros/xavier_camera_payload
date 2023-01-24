import argparse
import cv2
import json
import numpy as np
import os
import re
import sys
import rosbag
import rospy
import std_msgs.msg
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import logging
log = logging.getLogger("mkv-to-ros")
logging.basicConfig(level=os.environ.get("LOGLEVEL", "INFO"))

def is_valid_file(file_name):
    return os.path.isfile(file_name)

def time_message_to_ros_time(ros_time_message, time_field="tc"):
    time_split = ros_time_message.split("\n")

    fall_back_time = None
    for time_samp in time_split:
        if time_field in time_samp:
            #log.info(time_samp.split(" "))
            sec_val = time_samp.split(" ")[1]
            nsec_val = time_samp.split(" ")[-1]
            #log.info("TC: {0}.{1}".format(sec_val, nsec_val))
            return rospy.Time(secs=int(time_samp.split(" ")[1]), nsecs=int(time_samp.split(" ")[-1])) 

        elif "RT" in time_samp:
            sec_val = time_samp.split(" ")[1]
            nsec_val = time_samp.split(" ")[-1]
            #log.info("RT: {0}.{1}".format(sec_val, nsec_val))
            fall_back_time = rospy.Time(secs=int(time_samp.split(" ")[1]), nsecs=int(time_samp.split(" ")[-1])) 


    log.warn("TC Failed Using fall-back time for frame {0} ".format(time_split))

    if not fall_back_time:
        log.error("Fall back failed --- sorry TC interpolation not currently implemented")
        sys.exit(0)

    return fall_back_time

def rotate_image(img, rotation):
    if ( rotation != "none" ):
        if ( rotation == "left" ):
            rot_type = cv2.ROTATE_90_COUNTERCLOCKWISE
        elif ( rotation == "right" ):
            rot_type = cv2.ROTATE_90_CLOCKWISE
        elif ( rotation == "flip" ):
            rot_type = cv2.ROTATE_180
        else:
            raise Exception("Unexpected rotation type: ".format(rotation))


        return cv2.rotate( img, rot_type )
    else:
        return img

def frame_to_ros_msg(cv_frame, downsample, time_header, rotation="none"):
    width = int(cv_frame.shape[1] / downsample)
    height = int(cv_frame.shape[0] / downsample)
    dim = (width, height)
    resized_frame = cv2.resize(cv_frame, dim, interpolation = cv2.INTER_AREA)

    # Rotate the image.
    resized_frame = rotate_image( resized_frame, rotation )

    try:
        # Publish image.
        img_msg = bridge.cv2_to_imgmsg(resized_frame, "bgr8")
        img_msg.header.stamp = time_header
        return img_msg
    except CvBridgeError as err:
        print(err)

    return None

def parse_times_from_bag(input_bag):

    camera_frame_dict = {}


    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        
        if camera_time_root in topic:
            camera_id = topic.split("_")[-1]
            camera_time_ros_topic = camera_time_root + camera_id

            log.info("Camera Time: {0} ---> {1} ".format(t, msg.data))
            next_time = time_message_to_ros_time(msg.data)
            if not camera_time_ros_topic in camera_frame_dict:
                camera_frame_dict[camera_time_ros_topic] = []

            camera_frame_dict[camera_time_ros_topic].append(next_time)


    return camera_frame_dict

def get_first_integer(s):
    m = re.search(r'^(\d+)_', s)
    assert( m is not None ), "{} does not have a leading integer.".format(s)
    return m.group(1)

def parse_times_from_srt(all_srt_full_paths):

    camera_frame_dict = {}
    camera_srt_idx_dict = {} # The index in all_srt_full_paths

    for index, srt_file in enumerate(all_srt_full_paths):
        with open(srt_file) as infile:
            # camera_id = os.path.basename(srt_file).split("_")[0]
            camera_id = get_first_integer(os.path.basename(srt_file))
            log.info("Camera ID:  {0} {1} ".format(camera_id, srt_file))  
            all_lines = infile.readlines()   
            camera_frame_dict[camera_id] = []
            camera_srt_idx_dict[camera_id] = index
            #log.info("Read {0} lines of data ", str(len(all_lines)))       
            frame_block = ""
              
            for line in all_lines:
                #log.info("Reading line: " + line)
                if not line.strip():
                    next_time = time_message_to_ros_time(frame_block)
                    camera_frame_dict[camera_id].append(next_time)
                    frame_block = ""
                else:
                    frame_block+=line

    return camera_frame_dict, camera_srt_idx_dict

def convert_video_to_bag(video_full_path, 
    stamp_list, 
    output_topic, 
    downsample, 
    output_bag_path="/bag-data/saa-data/20201226/output_camera",
    rotation="none"):
    # Rotation configuration.
    if ( rotation == "none" ):
        # Get the first number of separated by "_" in video_full_path.
        prefix = get_first_integer(os.path.basename(video_full_path))

        # Try to read the JSON file.
        dir_video = os.path.dirname(video_full_path)
        conf_fn = os.path.join(dir_video, "%s.json" % (prefix))

        if ( os.path.isfile(conf_fn) ):
            log.info("Found configuration file %s. " % (conf_fn))
            # Read the JSON file.
            with open(conf_fn, "r") as fp:
                conf = json.load(fp)

            rotation = conf["rotation"]

    log.info("roation = %s" % (rotation))

    cap = cv2.VideoCapture(video_full_path)

    if not cap.isOpened():
        log.error("Error opening video {0} ".format(video_full_path))
        return

    log.info("Parsing Video! {0} ".format(video_full_path))

    frame_number = 1
    
    prev_time = 0.0
        
    for check_stamp in stamp_list:
        #print(check_stamp)
        next_time = stamp_list[frame_number-1]
        if prev_time and prev_time > next_time:
            log.error("Timing inconsistent! {0} {1} ".format(prev_time, next_time))
            sys.exit(0)
        prev_time = next_time

    bag_file_name = os.path.basename(video_full_path).replace(".mkv", ".bag")
    output_bag_path = os.path.join(output_bag_path, bag_file_name)
    log.info("Writing Bag {0} ".format(output_bag_path))
    
    with rosbag.Bag(output_bag_path, 'w') as outbag:

        while cap.isOpened():
            status, frame = cap.read()
            if status and frame_number<=len(stamp_list) :
                log.info("Read frame Success {0} ".format(frame_number))
                next_time = stamp_list[frame_number-1]
                ros_msg = frame_to_ros_msg(frame, downsample, next_time, rotation=rotation)
                ros_msg.header.seq = frame_number
                if prev_time and prev_time > ros_msg.header.stamp:
                   log.info("Timing is off! {0} {1} ".format(prev_time, ros_msg.header.stamp))
                
                log.info("Writing frame {0} at ROS Time: {1} ".format(frame_number, ros_msg.header))
             
                outbag.write(output_topic, ros_msg, ros_msg.header.stamp)
            else:
                log.info("Read frame FAILED! {0} ".format(frame_number))
                break
            frame_number+=1


bridge = CvBridge()

parser = argparse.ArgumentParser(description='Merge MKV/SRT output into ROS Bag')

parser.add_argument("-m", dest="video_file_path", required=True,
                    help="Folder containing all camera videos to be converted to ROS Bag")
parser.add_argument("-b", dest="input_bag", required=False,
                    help="input ROSBag containing the other recorded ROS topics from the OpenResearchDrone")
parser.add_argument("-c", dest="closed_caption_path", required=True,
                    help="directory containing all closed caption files ")
parser.add_argument("-o", dest="output_path", required=True,
                    help="Path to write the output ROS Bags constructed from the MKV/SRT")
parser.add_argument("-ds", dest="downsample", required=True, type=int, default=3,
                    help="the downsample times of the original MKV/SRT images")
parser.add_argument("-rot", dest="rotation", required=False, type=str, default="none", 
                    help="the rotation flag. none: no rotation or use the configuration file if it presents, left: counter-clockwise, right: clockwise, flip: 180 degrees.")

args = parser.parse_args()

is_fail = False

extensions = [".mkv"]
all_video_full_paths = []
all_srt_full_paths = []

if not os.path.exists(args.video_file_path):
    log.info("Error, {} does not seem to exist or access permissions are bad. ".format(args.video_file_path))
    is_fail = True
else:
    for subdir, dirs, files in os.walk(args.video_file_path):
        for file in files:
            ext = os.path.splitext(file)[-1].lower()
            if ext in extensions:
                log.info("Adding video to processing queue! {0}".format(file))
                all_video_full_paths.append(os.path.join(subdir, file))

if not os.path.exists(args.closed_caption_path):
    log.info("Error, {} does not seem to exist or access permissions are bad. ".format(args.closed_caption_path))
    is_fail = True
else:
    for subdir, dirs, files in os.walk(args.closed_caption_path):
        for file in files:
            ext = os.path.splitext(file)[-1].lower()
            if ext == ".srt":
                log.info("Adding caption to processing queue! {0}".format(file))
                all_srt_full_paths.append(os.path.join(subdir, file))

all_srt_full_paths.sort()
all_video_full_paths.sort()
log.info(all_srt_full_paths)
log.info(all_video_full_paths)

# Don't care about the bag since it is optional
#if not is_valid_file(args.input_bag):
#    log.error("Error, file does not seem to exist! ").format(args.input_bag)
#    is_fail = True

if is_fail:
    log.error("Cannot combine files, please input correct paths or fix permission issue.")
    sys.exit(0)

if not os.path.exists(args.output_path):
    os.makedirs(args.output_path)

if not os.path.exists(args.output_path):
    log.error("Output path does not exist, and could not create it. {0} ".format(args.output_path))
    sys.exit(0)

log.info("Combining {0} using {1} as a reference for sync ".format(args.video_file_path, args.input_bag))

# Loop Through source bag --- TC -> ROSTime
# Get Frame -- TC_F 

camera_time_root = "/ros_deep_frame_time/camera_"
camera_topic_root = "/ros_frame/camera_"
        
# bag = rosbag.Bag('test.bag', 'w')

if False:
    camera_frame_dict, _ = parse_times_from_bag(args.input_bag)
    for camera_frame_topic in camera_frame_dict.keys():
        camera_id = camera_frame_topic.split("_")[-1]
        log.info(" ID: {0} Frames: {1} ".format(camera_id, len(camera_frame_dict[camera_frame_topic])))
        convert_video_to_bag(
            all_video_full_paths[int(camera_id)], 
            camera_frame_dict[camera_frame_topic], 
            camera_frame_topic, 
            args.downsample, 
            args.output_path,
            args.rotation)

else:
    camera_frame_dict, camera_srt_idx_dict = parse_times_from_srt(all_srt_full_paths)
    for camera_id in camera_frame_dict.keys():
        log.info(" ID: {0} Frames: {1} ".format(camera_id, len(camera_frame_dict[camera_id])))
        print(len(all_video_full_paths))
        convert_video_to_bag(
            all_video_full_paths[camera_srt_idx_dict[camera_id]*2 + 1], 
            camera_frame_dict[camera_id], 
            camera_topic_root+camera_id, 
            args.downsample, 
            args.output_path,
            args.rotation)