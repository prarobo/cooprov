#!/usr/bin/env python

""" Tf publishing node for openrov
This node publishes the tf tree"""
 
import roslib

import rospy
import tf

class openrovTF(object):
    def __init__(self):
        '''Constructor'''
        rospy.init_node('openrov_tf_broadcaster')
        self.br = tf.TransformBroadcaster()
        
    def publishBaseTF(self):
        self.br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "base_link",
                         "world")
        
    def publishCameraTF(self):
        self.br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "minoru_link",
                         "base_link")
        

if __name__ == '__main__':
    myOpenrovTF = openrovTF()
    rate = rospy.Rate(10.0)
    print "Publishing openrov tf tree ..."
    
    while not rospy.is_shutdown():
        myOpenrovTF.publishBaseTF()
        myOpenrovTF.publishCameraTF()
        rate.sleep()
    
