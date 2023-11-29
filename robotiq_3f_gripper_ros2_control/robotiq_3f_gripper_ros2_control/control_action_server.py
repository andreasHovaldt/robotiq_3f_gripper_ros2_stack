# Basic ROS2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import ReliabilityPolicy, QoSProfile


# Executor and callback imports
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ROS2 interfaces
from robotiq_3f_gripper_ros2_interfaces.msg import Robotiq3FGripperInputRegisters, Robotiq3FGripperOutputRegisters
from robotiq_3f_gripper_ros2_interfaces.action import Robotiq3FGripperPositionGoal


# Others
import numpy as np
import time, threading, math
from pymodbus.client import ModbusTcpClient



class GripperControlListener(Node):
    '''
    Notes:
    * Subscribes to the topic Robotiq3FGripper/OutputRegisters, and sends the values as a command to the Robotiq 3F Gripper (Using "Robotiq3FGripperOutputRegisters" msg)
    * Publishes status of gripper to the Robotiq3FGripper/InputRegisters topic (Using "Robotiq3FGripperInputRegisters" msg)
    '''
    def __init__(self):
        super().__init__("gripper_control_listener_node")
        rclpy.logging.set_logger_level('gripper_control_listener_node', rclpy.logging.LoggingSeverity.INFO)
        
        # Declare parameters for node
        self.declare_parameter("gripper_address", "172.31.1.69")
        address = self.get_parameter("gripper_address").get_parameter_value().string_value
        
        # Variable Init
        self.gripper_connection_init(address)
        self.output_register_command = [11,0,0,0,128,0]  #[action_request,0,0,start_pos,start_speed,start_force]
        self.lock = threading.Lock() # Used for secure comm to gripper?

        # Callback groups
        self.group_1 = MutuallyExclusiveCallbackGroup() # read output register subscriber
        self.group_2 = MutuallyExclusiveCallbackGroup() # gripper read input registers timer
        self.group_3 = MutuallyExclusiveCallbackGroup() # action group
        
        # msgs init
        self.input_registers = Robotiq3FGripperInputRegisters()
        self.output_registers = Robotiq3FGripperOutputRegisters()
        
        # Publishers
        self._input_register_pub = self.create_publisher(Robotiq3FGripperInputRegisters, "Robotiq3FGripper/InputRegisters", 10)
        
        # Subscribers
        self._output_register_sub = self.create_subscription(Robotiq3FGripperOutputRegisters, 'Robotiq3FGripper/OutputRegisters', self.read_output_registers, QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE), callback_group=self.group_1)

        # Timers
        self._read_input_timer = self.create_timer(0.1, self.read_input_registers, callback_group=self.group_2)
        
        # Actions
        self._gripper_action_server = self.crea


    def gripper_connection_init(self, address):
        self.client = ModbusTcpClient(address)
        self.client.connect()
            
    
    def send_register_msg(self):
        while True:
            self.send_data(self.output_register_command) # Send current registers_msg to gripper
            time.sleep(0.5) # Let the gripper process the msg for the given amount of time

            gSTA = self.input_registers.g_sta
            gIMC = self.input_registers.g_imc

            if gSTA != 0 and gIMC == 3: # Check the gripper activation, mode, and position has reached stop point
                self.get_logger().info("Register message successfully completed")
                break
    
    
    def send_data(self, data):   
        """Send a command to the Gripper - the method takes a list of uint8 as an argument. 
        The meaning of each variable depends on the Gripper model (see support.robotiq.com for more details)"""

        # Make sure data has an even number of elements   
        if(len(data) % 2 == 1):
            data.append(0)   
        
        # Initiate message as an empty list
        message = []    
        
        # Fill message by combining two bytes in one register
        for i in range(0, int(len(data)/2)):
            message.append((data[2*i] << 8) + data[2*i+1])
        
        # print(f"write_registers({message})") # Debug  
        #To do!: Implement try/except
        with self.lock:
            self.client.write_registers(0, message)
    
    
    def read_output_registers(self, output_registers_msg):
        
        '''
        Class function for reading output registers and setting the output command using the Robotiq3FGripperOutputRegisters message type
        '''
        
        print("Setting output command for Gripper")
        #old_output_register_command = self.output_register_command
        
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
        
        #if self.output_register_command != old_output_register_command:
        if self.output_register_command[0] != 0:
            self.send_register_msg()
        
    
    def read_input_registers(self, numBytes=4):
        
        '''
        Class function for reading input registers from gripper and publishing this to the input register topic using the Robotiq3FGripperInputRegisters message type
        '''
        
        numRegs = int(math.ceil(numBytes/2))

        with self.lock:
            response = self.client.read_input_registers(0,numBytes)
        
        response_byte_array = []

        for i in range(0, numRegs): # Splits the 16 bit registers from pymodbus into 2x 8 bit registers (uint8) 
            response_byte_array.append((response.getRegister(i) & 0xFF00) >> 8)
            response_byte_array.append(response.getRegister(i) & 0x00FF)
            
        
        # Setting Gripper status register
        response_byte0_bit_string = format(response_byte_array[0], '08b') # Format int to 8 bits
        
        self.input_registers.g_act = int(response_byte0_bit_string[7], base=2)   # gACT ->       Initialization status       -> 1 = Gripper activation
        self.input_registers.g_mod = int(response_byte0_bit_string[5:7], base=2) # gMOD ->       Operation mode status       -> 11 = Done and ready
        self.input_registers.g_gto = int(response_byte0_bit_string[4], base=2)   # gGTO -> Activation and mode change status -> 11 = Done and ready
        self.input_registers.g_imc = int(response_byte0_bit_string[2:4], base=2) # gIMC -> Activation and mode change status -> 11 = Done and ready
        self.input_registers.g_sta = int(response_byte0_bit_string[0:2], base=2) # gSTA ->           Motion status           -> 11 = Completed successfully
        
        # Setting Fault status register
        self.input_registers.g_flt = response_byte_array[2]
        
        # Setting Position request echo register
        self.input_registers.g_pra = response_byte_array[3]
        
        # Publish the read input registers
        self._input_register_pub.publish(self.input_registers)
    
    
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