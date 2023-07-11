from time import sleep
import rclpy                                     # ROS2 Python Client Library
from rclpy.node import Node                      # ROS2 Node
from sensor_msgs.msg import Joy                  # ROS2 standard Joy Message
import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu


class ManualMode(Node):

    mode = 0  # 0: Null, 1: Relative Position Control, 2: Aboslute Position Control, 3: Velocity Control, 4: Torque Control
    state = 7  # 0: Null, 5: Emergency Stop, 6: Clear Alarm, 7: Disable, 8: Enable

    motors_x = 0
    motors_y = 0
    left_motor_param = 0
    right_motor_param = 0

    state_enable = [False, 0, 0]  # [state_enable, previous_button_pressed, button_pressed]
    state_emerge = 0

    max_speed = 25
    twist_ratio = 0.5

    sleep_time = 0.02


    def __init__(self,name):
        super().__init__(name)

        self.rs485_1 = modbus_rtu.RtuMaster(serial.Serial(port="/dev/ttySC0", baudrate=115200, bytesize=8, parity='N', stopbits=1, xonxoff=0))
        self.rs485_1.set_timeout(0.5)
        self.rs485_1.set_verbose(True)
        sleep(1)
        self.esc_clear_alarm()
        self.esc_disable()
        self.esc_velocity_control()

        self.sub_joy = self.create_subscription(
            Joy, 
            "joy", 
            self.joy_recv, 
            10)
        
    def esc_clear_alarm(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=6)
        # sleep(self.sleep_time)

    def esc_emergency_stop(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=5)
        # sleep(self.sleep_time)

    def esc_enable(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=8)
        # sleep(self.sleep_time)

    def esc_disable(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=7)
        # sleep(self.sleep_time)

    def esc_torque_control(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200d), output_value=4)
        # sleep(self.sleep_time)

    def esc_velocity_control(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200d), output_value=3)
        # sleep(self.sleep_time)

    def esc_motor_velocity(self, left_target, right_target):
        self.rs485_1.execute(1, cst.WRITE_MULTIPLE_REGISTERS, int(0x2088), output_value=[-left_target, right_target])
        # sleep(self.sleep_time)

            

    def joy_recv(self,msg):
        input = {
            'left_joy_x'              : -msg.axes[0]     ,
            'left_joy_y'              :  msg.axes[1]     ,
            'right_joy_x'             : -msg.axes[2]     ,
            'right_joy_y'             :  msg.axes[3]     ,
            'dpad_x'                  : -msg.axes[6]     ,
            'dpad_y'                  :  msg.axes[7]     ,
            'left_shoulder_button'    :  msg.buttons[6]  ,
            'right_shoulder_button'   :  msg.buttons[7]  ,
            'left_shoulder_button_2'  :  msg.buttons[8]  ,
            'right_shoulder_button_2' :  msg.buttons[9]  ,
            'a_button'                :  msg.buttons[0]  ,
            'b_button'                :  msg.buttons[1]  ,   
            'x_button'                :  msg.buttons[3]  ,
            'y_button'                :  msg.buttons[4]  ,
            'select_button'           :  msg.buttons[10]  ,
            'start_button'            :  msg.buttons[11]  ,
            'xbox_button'             :  msg.buttons[12]  ,
            'left_joy_button'         :  msg.buttons[13]  ,
            'right_joy_button'        :  msg.buttons[14]  
        } 
        self.input_processor(input)

    def input_processor(self,input):
        self.state_emerge = input['left_shoulder_button']

        self.state_enable[1] = self.state_enable[2]
        self.state_enable[2] = input['right_shoulder_button']
        self.motors_x = input['left_joy_x']
        self.motors_y = input['left_joy_y']

        if self.state_enable[1] == 0 and self.state_enable[2] == 1:
            self.state_enable[0] = not self.state_enable[0]
            if self.state_enable[0] == True:
                self.esc_clear_alarm()
                self.esc_velocity_control()
                self.esc_enable()
                self.get_logger().info("Motors Enabled")
            else:
                self.esc_clear_alarm()
                self.esc_velocity_control()
                self.esc_disable()
                self.get_logger().info("Motors Disabled")

        if self.state_enable[0] == True:
            self.left_motor_param = int(self.max_speed * (self.motors_y + self.motors_x * self.twist_ratio))
            self.right_motor_param = int(self.max_speed * (self.motors_y - self.motors_x * self.twist_ratio))

            self.esc_motor_velocity(self.left_motor_param, self.right_motor_param)
            self.get_logger().info("Left Motor: %d, Right Motor: %d" % (self.left_motor_param, self.right_motor_param))


def main(args=None):
    rclpy.init(args=args)
    node = ManualMode('manual_mode')
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()
