from time import sleep
import os
import rclpy                                     # ROS2 Python Client Library
from rclpy.node import Node                      # ROS2 Node
from sensor_msgs.msg import Joy                  # ROS2 standard Joy Message
import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from pydub import AudioSegment
from pydub.playback import play
import can
import RPi.GPIO as GPIO
import math

class FixedPosition(Node):
    
    mode = 0  # 0: Null, 1: Relative Position Control, 2: Aboslute Position Control, 3: Velocity Control, 4: Torque Control
    state = 7  # 0: Null, 5: Emergency Stop, 6: Clear Alarm, 7: Disable, 8: Enable

    motors_x = 0
    motors_y = 0
    left_motor_param = 0
    right_motor_param = 0

    state_enable = [False, 0, 0]  # [state_enable, previous_button_pressed, button_pressed]
    state_emerge = 0

    max_speed = 300
    twist_ratio = 0.5

    sleep_time = 0.02

    air_pump = 22
    air_valve = 23
    ac_input = 27

    axis_state = [0, 0]

    valve_state = [False, 0, 0]

    arm_enable = "1\n"
    arm_disable = "2\n"
    arm_down = "3\n"
    arm_up = "4\n"

    def __init__(self,name):
        super().__init__(name)
        # self.hillside = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/hillside.mp3', format='mp3')
        self.initialized = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/Initialized.mp3', format='mp3')
        self.enabled = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/Enabled.mp3', format='mp3')
        # self.disabled = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/Disabled.mp3', format='mp3')
        # self.emergency = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/EmergencyStop.mp3', format='mp3')
        self.stopped = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/Stopped.mp3', format='mp3')
        # self.fence = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/Fence.mp3', format='mp3')
        # self.stairs = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/Stairs.mp3', format='mp3')
        self.get_logger().info("Audio loaded.")

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.air_pump, GPIO.OUT)
        GPIO.setup(self.air_valve, GPIO.OUT)
        GPIO.setup(self.ac_input, GPIO.IN)
        self.get_logger().info("Air pump and valve initialized.")

        self.rs485_1 = modbus_rtu.RtuMaster(serial.Serial(port="/dev/ttySC0", baudrate=115200, bytesize=8, parity='N', stopbits=1, xonxoff=0))
        self.rs485_1.set_timeout(0.5)
        self.rs485_1.set_verbose(True)
        self.get_logger().info("RS485-1 initialized.")
        sleep(1)
        self.esc_clear_alarm()
        self.esc_disable()
        self.esc_position_control()
        self.get_logger().info("ESC initialized.")

        self.can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan')
        self.platform_raise = can.Message(arbitration_id=0x00000100, data=[0xf6, 0x00, 0x03, 0xe8, 0x0f, 0xa0, 0x00, 0x6b], is_extended_id=True)
        self.platform_lower = can.Message(arbitration_id=0x00000100, data=[0xf6, 0x01, 0x03, 0xe8, 0x0f, 0xa0, 0x00, 0x6b], is_extended_id=True)
        self.platform_stop = can.Message(arbitration_id=0x00000100, data=[0xf6, 0x00, 0x03, 0xe8, 0x00, 0x00, 0x00, 0x6b], is_extended_id=True)
        self.get_logger().info("CAN initialized.")

        self.arduino = serial.Serial('/dev/tty_ARM', 9600, timeout=0.1)
        self.get_logger().info("Arduino initialized.")

        self.sub_joy = self.create_subscription(
            Joy, 
            "joy", 
            self.joy_recv, 
            10)
        
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
        
    def esc_clear_alarm(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=6)

    def esc_emergency_stop(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=5)

    def esc_enable(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=8)

    def esc_disable(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=7)

    def esc_torque_control(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200d), output_value=4)

    def esc_velocity_control(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200d), output_value=3)

    def esc_position_control(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200d), output_value=1)
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200f), output_value=0)
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x2080), output_value=1500)
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x2081), output_value=1500)
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x2082), output_value=1500)
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x2083), output_value=1500)
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x208E), output_value=20)
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x208F), output_value=20)

    def esc_motor_velocity(self, left_target, right_target):
        self.rs485_1.execute(1, cst.WRITE_MULTIPLE_REGISTERS, int(0x2088), output_value=[left_target, -right_target])

    def esc_motor_position(self, left_target, right_target):
        left_regs = self.number_seprate(left_target)
        right_regs = self.number_seprate(-right_target)
        self.rs485_1.execute(1, cst.WRITE_MULTIPLE_REGISTERS, int(0x208A), output_value=[left_regs[0], left_regs[1], right_regs[0], right_regs[1]])
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=int(0x11))
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=int(0x12))

    def number_seprate(self, number):
        bytes_rep = number.to_bytes(4, byteorder='big', signed=True)
        high_reg = int.from_bytes(bytes_rep[:2], byteorder='big', signed=True)
        low_reg = int.from_bytes(bytes_rep[2:], byteorder='big', signed=True)
        return [high_reg, low_reg]
    
    def move_straight(self, distance):
        pulses = int(distance / (math.pi * 0.107) * 1024)
        self.esc_motor_position(pulses, pulses)

    def turn(self, angle):
        pulses = int(angle / 360 * (0.365 * math.pi / (math.pi * 0.107) * 1024))
        self.esc_motor_position(pulses, -pulses)

    def input_processor(self,input):
        self.state_emerge = input['left_shoulder_button']
        if self.state_emerge == 1:
            self.esc_emergency_stop()
            self.state_enable[0] = False
            play(self.emergency)
            self.get_logger().info("Emergency Stop")
        else:
            self.state_enable[1] = self.state_enable[2]
            self.state_enable[2] = input['right_shoulder_button']
            if self.state_enable[1] == 0 and self.state_enable[2] == 1:
                self.state_enable[0] = not self.state_enable[0]
                if self.state_enable[0] == True:
                    self.esc_clear_alarm()
                    self.esc_position_control()
                    self.esc_enable()
                    self.get_logger().info("Motors Enabled")
                    play(self.enabled)
                else:
                    self.esc_clear_alarm()
                    self.esc_position_control()
                    self.esc_disable()
                    self.get_logger().info("Motors Disabled")
                    play(self.stopped)

            if self.state_enable[0] == True:
                if input['dpad_x'] == -1:
                    self.move_straight(3)
                    sleep(20)
                    self.turn(180)
                    sleep(20)
                    self.move_straight(3)
                    sleep(20)
                    self.arduino.write(self.arm_enable.encode())
                    self.arduino.write(self.arm_down.encode())
                    sleep(20)
                    GPIO.output(self.air_pump, GPIO.HIGH)
                    sleep(7)
                    GPIO.output(self.air_pump, GPIO.LOW)
                    sleep(1)
                    self.arduino.write(self.arm_up.encode())
                    sleep(20)
                    self.arduino.write(self.arm_disable.encode())
                    GPIO.output(self.air_valve, GPIO.HIGH)
                    sleep(1)
                    GPIO.output(self.air_valve, GPIO.LOW)


def main(args=None):
    rclpy.init(args=args)
    node = FixedPosition('fixed_position')
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()
