#! /usr/bin/env python3
# add robot transform info by gazebo ModelState msg

import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

from nav_msgs.msg import Odometry


class OdomTFConverter:
    def __init__(self, msg_name):
        self.update_stamp = rospy.Time.now().to_sec()
        self.odom_sub_ = rospy.Subscriber(msg_name, Odometry, self.OdomCallBack)
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
    
    def OdomCallBack(self, data):
        cur_stamp = data.header.stamp
        
        if cur_stamp.to_sec() < self.update_stamp + 0.002:
            # do not update too fast
            return
        
        t = geometry_msgs.msg.TransformStamped()
        t.header = data.header
        t.child_frame_id = data.child_frame_id
        t.transform.translation = data.pose.pose.position
        t.transform.rotation = data.pose.pose.orientation
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)
        
        self.update_stamp = cur_stamp.to_sec()
        return
    


if __name__ == '__main__':
    rospy.init_node('odom_to_tf')
    msg_name = 'odom'
    converter = OdomTFConverter(msg_name)
    
    rospy.spin()

