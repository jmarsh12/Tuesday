# Copyright 2019 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A demo that runs object detection on camera frames using OpenCV.

TEST_DATA=../all_models

Run face detection model:
python3 detect.py \
  --model ${TEST_DATA}/mobilenet_ssd_v2_face_quant_postprocess_edgetpu.tflite

Run coco model:
python3 detect.py \
  --model ${TEST_DATA}/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite \
  --labels ${TEST_DATA}/coco_labels.txt

"""
import argparse
import cv2
import os

import rospy
from std_msgs.msg import String

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

def main():
    ##### Given by TensorFlow example ####
    default_model_dir = '/home/pi/ros_catkin_ws/src/Tuesday/tensorflow_models/'
    default_model = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
    default_labels = 'coco_labels.txt'
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help='.tflite model path',
                        default=os.path.join(default_model_dir,default_model))
    parser.add_argument('--labels', help='label file path',
                        default=os.path.join(default_model_dir, default_labels))
    parser.add_argument('--top_k', type=int, default=3,
                        help='number of categories with highest score to display')
    parser.add_argument('--camera_idx', type=int, help='Index of which video source to use. ', default = 0)
    parser.add_argument('--threshold', type=float, default=0.1,
                        help='classifier score threshold')
    args, unknown = parser.parse_known_args()
    
    print('Loading {} with {} labels.'.format(args.model, args.labels))
    interpreter = make_interpreter(args.model)
    interpreter.allocate_tensors()
    labels = read_label_file(args.labels)
    inference_size = input_size(interpreter)

    cap = cv2.VideoCapture(args.camera_idx)

    if cap.isOpened():
        # start ros
        object_detector = RosOpenCVWatcher(cap=cap, labels=labels, inference_size=inference_size, threshold=args.threshold, top_k=args.top_k, interpreter=interpreter)
        object_detector.start_spinning()

    #     ret, frame = cap.read()
    #     if not ret:
    #         break
    #     cv2_im = frame

    #     cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
    #     cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
    #     run_inference(interpreter, cv2_im_rgb.tobytes())
    #     objs = get_objects(interpreter, args.threshold)[:args.top_k]
    #     cv2_im = append_objs_to_img(cv2_im, inference_size, objs, labels)

    #     cv2.imshow('frame', cv2_im)
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break

    cap.release()
    cv2.destroyAllWindows()
    
def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
        bbox = obj.bbox.scale(scale_x, scale_y)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)

        percent = int(100 * obj.score)
        label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

        cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2_im = cv2.putText(cv2_im, label, (x0, y0+30),
                             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    return cv2_im

########################################################
# Functions for displaying output to ROS
########################################################

class RosOpenCVWatcher():
    opencv_publisher = None

    def get_ratio_coords(self, obj, cv2_im):
        height, width, channels = cv2_im.shape
        scale_x, scale_y = width / self.inference_size[0], height / self.inference_size[1]

        # get pixel coordinates
        # TODO: Optimize this
        bbox = obj.bbox.scale(scale_x, scale_y)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)

        # get center val
        center_x = (x0 + x1) // 2
        center_y = (y0 + y1) // 2

        # convert pixel coordinates to ratio coordinates (0-1)
        ratio_y = center_y / height
        ratio_x = center_x / width
        print("Center x: ", center_x)
        print("Center y: ", center_y)
        print("Scale x: ", scale_x)

        return ratio_x, ratio_y
        
    
    def __init__(self, node_name="opencv_find", subscribe_to='voice_commands',
                 publish_to="opencv_coordinates", cap=None, labels=None, inference_size=None,
                 threshold=None, top_k=None, interpreter=None):
        # create rospy node and subscribe to voice_commands
        rospy.init_node(node_name, anonymous = True)
        rospy.Subscriber(subscribe_to, String, callback=self.find_object_callback)
        self.opencv_publisher = rospy.Publisher(publish_to, String, queue_size=10)

        # TODO: add videocapture to the list of parameters
        if cap is None:
            self.cap = cv2.VideoCapture(0)
        else:
            self.cap = cap

        self.labels = labels
        self.inference_size = inference_size
        self.threshold = threshold
        self.top_k = top_k
        self.interpreter=interpreter


    def find_object_callback(self, data):
        object_to_find = data.data
        print("Finding Object: ", object_to_find)

        ################ tensorstuff ##########
        
        ret, frame = self.cap.read()
        if not ret:
            print("find_object: Capture image failed...")
            return
        cv2_im = frame

        cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        cv2_im_rgb = cv2.resize(cv2_im_rgb, self.inference_size)
        run_inference(self.interpreter, cv2_im_rgb.tobytes())
        objs = get_objects(self.interpreter, self.threshold)[:self.top_k]
        cv2_im = append_objs_to_img(cv2_im, self.inference_size, objs, self.labels)

        # TODO Soon: find the object with the highest confidence and get that. 
        # TODO: Find an object by name. For example, user says, "find pen" and it searches
        #       for a pen. For now, find only the first object. 
        if len(objs) == 0:
            print("No object!")
            # TODO: Speak, "object not found"
            return
        obj_to_find = objs[0]
        ratio_x, ratio_y = self.get_ratio_coords(obj_to_find, cv2_im)
        coords = str(ratio_x) + ',' + str(ratio_y)
        
        # share the object's location
        self.opencv_publisher.publish(coords)

        # show the fram efor 2000 ms
        print("Showing image")
        # cv2.imshow('frame', cv2_im)
        # cv2.waitKey(1)

        

    def publish_coords(self, center_x, center_y):
        if opencv_publisher is None:
            print("Error: opencv_publisher is not yet defined")
            return

        coords = str(center_x) + ',' + str(center_y)

        # publish coordinates to opencv_coordinates channel
        opencv_publisher.publish(coords)


    def start_spinning(self):
        
        rospy.spin()
    

if __name__ == '__main__':
    main()
