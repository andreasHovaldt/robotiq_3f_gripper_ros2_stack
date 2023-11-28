# Basic ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile


# Executor and callback imports
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ROS2 interfaces
from robotiq_3f_gripper_ros2_interfaces.msg import Robotiq3FGripperInputRegisters, Robotiq3FGripperOutputRegisters


# Others
import numpy as np
import time



class GripperControlListener(Node):
    '''
    Notes:
    * Subscribes to the topic Robotiq3FGripper/OutputRegisters, and sends the values as a command to the Robotiq 3F Gripper (Using "Robotiq3FGripperOutputRegisters" msg)
    * Publishes status of gripper to the Robotiq3FGripper/InputRegisters topic (Using "Robotiq3FGripperInputRegisters" msg)
    '''
    def __init__(self):
        super().__init__("gripper_control_listener_node")
        rclpy.logging.set_logger_level('yolov8_node', rclpy.logging.LoggingSeverity.INFO)
        
        ## Declare parameters for node
        # self.declare_parameter("model", "yolov8s-seg.pt")
        # model = self.get_parameter("model").get_parameter_value().string_value
        
                                    #  [action_request,0,0,start_pos,start_speed,start_force]
        self.output_register_command = [0,0,0,0,128,0]

        
        
        
        # Callback groups
        self.group_1 = MutuallyExclusiveCallbackGroup() # output register subscriber
        self.group_2 = MutuallyExclusiveCallbackGroup() # gripper command timer
        
        # msgs
        self.input_registers = Robotiq3FGripperInputRegisters()
        self.output_registers = Robotiq3FGripperOutputRegisters()
        
        # Publishers
        self._input_register_pub = self.create_publisher(Robotiq3FGripperInputRegisters, "Robotiq3FGripper/InputRegisters", 10)
        
        # Subscribers
        self._output_register_sub = self.create_subscription(Robotiq3FGripperOutputRegisters, 'Robotiq3FGripper/OutputRegisters', self.output_register_callback, QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE), callback_group=self.group_1)

        # Timers
        self._gripper_command_timer = self.create_timer(0.5, self.gripper_command, callback_group=self.group_2) # 25 hz


    
    def output_register_callback(self, msg):
        self.set_output_command(msg)
    
    
    def gripper_command(self):
        #self.get_logger().info("Timer cb")
        pass
    
    
    
    def set_output_command(self, output_registers_msg: Robotiq3FGripperOutputRegisters): # byte 0 rATR: int, rGTO: int, rMOD: int, rACT: int
        '''
        Class function for setting the output command using the Robotiq3FGripperOutputRegisters message type
        '''
        
        print("Setting output command for Gripper")
        
        rACT = output_registers_msg.r_act # Activation
        rMOD = output_registers_msg.r_mod # Mode
        rGTO = output_registers_msg.r_gto # Go to pos
        rATR = output_registers_msg.r_atr # Auto release
        
        byte_0_bits = f"000{rATR}{rGTO}0{rMOD}{rACT}"
        byte_0_int = int(byte_0_bits, base=2)
        self.output_register_command[0] = byte_0_int
        
        
        rPRA = output_registers_msg.r_pra # Target pos
        rSPA = output_registers_msg.r_spa # Grip speed
        rFRA = output_registers_msg.r_fra # Grip force
        
        self.output_register_command[3] = rPRA
        self.output_register_command[4] = rSPA
        self.output_register_command[5] = rFRA
    
    
    
    
    
    
    
    
    
    def shutdown_callback(self):
        self.get_logger().warn("Shutting down...")
        
        
        
        
def main(args=None):
    rclpy.init(args=args)

    # Instansiate node class
    ControlListenerNode = GripperControlListener()

    # Create executor
    executor = MultiThreadedExecutor()
    executor.add_node(ControlListenerNode)

    
    try:
        # Run executor
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    
    finally:
        # Shutdown executor
        ControlListenerNode.shutdown_callback()
        executor.shutdown()




if __name__ == "__main__":
    main()