#!/usr/bin/python

# This node listens to a joystick.  When a particular button is pressed,
# it publishes an empty message to a specified topic

import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

class JoyTrigger:
    def __init__(self):
        # Get joy button from parameter server
        self.button_id = rospy.get_param("~button_number", 0) #button X

        # Set up trigger publisher
        trigger_topic = rospy.get_param("~trigger_topic", "stop_trigger")
        self.trigger_pub = rospy.Publisher(trigger_topic, Empty, queue_size=1)

        # Subscribe to Joystick topic
        
        rospy.Subscriber('husky_a1/joy', Joy, self.joy_callback)

    def joy_callback(self, joy):
        if joy.buttons[self.button_id] == True:
            self.trigger_pub.publish(Empty())

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('joy_trigger')

    # Create the Joy Trigger object
    JoyTrigger()
    
    # Wait for messages on topics, go to callback function when new messages arrive.
    rospy.spin()
