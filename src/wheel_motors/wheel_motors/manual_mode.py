from re import T
from time import sleep
import rclpy                                     # ROS2 Python Client Library
from rclpy.node import Node                      # ROS2 Node
from sensor_msgs.msg import Joy                  # ROS2 standard Joy Message
from share.msg import Send485


class ManualMode(Node):
    

    
     def __init__(self,name):
        super().__init__(name)

        self.sub_joy = self.create_subscription(
            Joy, 
            "joy", 
            self.joy_recv, 
            10)
            

    def joy_recv(self,msg):
        input = {
            'left_joy_x'            : -msg.axes[0]     ,
            'left_joy_y'            :  msg.axes[1]     ,
            'right_joy_x'           : -msg.axes[2]     ,
            'right_joy_y'           :  msg.axes[3]     ,
            'dpad_x'                : -msg.axes[6]     ,
            'dpad_y'                :  msg.axes[7]     ,
            'left_trigger'          : -msg.axes[5]     ,
            'right_trigger'         : -msg.axes[4]     ,
            'left_shoulder_button'  :  msg.buttons[6]  ,
            'right_shoulder_button' :  msg.buttons[7]  ,
            'a_button'              :  msg.buttons[0]  ,
            'b_button'              :  msg.buttons[1]  ,   
            'x_button'              :  msg.buttons[3]  ,
            'y_button'              :  msg.buttons[4]  ,
            'select_button'         :  msg.buttons[15]  ,
            'start_button'          :  msg.buttons[11]  ,
            'xbox_button'           :  msg.buttons[16]  ,
            'left_joy_button'       :  msg.buttons[13]  ,
            'right_joy_button'      :  msg.buttons[14]  
        } 
        self.input_processor(input)