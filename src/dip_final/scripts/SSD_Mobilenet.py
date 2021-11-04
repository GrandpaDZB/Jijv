#!/usr/bin/env python3
#coding:utf‐8 #设置编码格式为utf‐8 

import tensorflow_hub as hub
import cv2
import numpy as np
import tensorflow as tf
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading


use_PersonTracker = False
diff_msg = Float32()

src = []
def Callback_usePersonTracker(data):
    global use_PersonTracker
    use_PersonTracker = data.data
    return
def Callback_image(data):
    global src
    #global bridge
    bridge = CvBridge()
    src = bridge.imgmsg_to_cv2(data, "bgr8")
    return
def thread_spin():
    rospy.spin()


detector = hub.load("/home/grandpadzb/tfhub_modules/ssd_mobilenet_v2_2")
print("Complete loading")

rospy.init_node('Person_traker', anonymous=True)
sub_use_PersonTraker = rospy.Subscriber("/dip_final/use_PersonTracker", Bool, Callback_usePersonTracker)
sub_image = rospy.Subscriber("/dip_final/image", Image, Callback_image)
pub_diff = rospy.Publisher("/dip_final/diff", Float32, queue_size=10);

add_thread = threading.Thread(target=thread_spin)
add_thread.start()

shrink_rate_x = 0
shrink_rate_y = 0
diff = 0

rate = rospy.Rate(50)
# while cv2.waitKey(1) != 113:
while not rospy.is_shutdown():
    rate.sleep()
    # running when use_PersonTracker == True
    if use_PersonTracker and src != []:
        if shrink_rate_x == 0 or shrink_rate_y == 0:
            shrink_rate_x = 320.0/src.shape[0]
            shrink_rate_y = 320.0/src.shape[1]

        tmp_src = cv2.resize(src, dsize=(320,320))
        tmp_src = tmp_src[np.newaxis,:]
        img = tf.convert_to_tensor(tmp_src, dtype="uint8")

        output = detector(img)
        for i in range(100):
            class_index = output["detection_classes"].numpy()[0][i]
            if class_index == 1.0:
                box = np.fix(output["detection_boxes"].numpy()[0][i]*320)
                cv2.rectangle(
                    src, 
                    (int(box[1]/shrink_rate_y),int(box[0]/shrink_rate_x)), 
                    (int(box[3]/shrink_rate_y),int(box[2]/shrink_rate_x)), 
                    (255,255,255), 
                    2
                )
                diff = (box[3]+box[1])/shrink_rate_y/2-160/shrink_rate_y
                break
        # cv2.imshow("result", src)
        # rospy.loginfo(f'diff = {diff}')
        diff_msg.data = diff
        pub_diff.publish(diff_msg)
    else:
        pass
cv2.destroyAllWindows()
exit(0)







