# Basic ROS2
import rclpy
from rclpy.node import Node

# Executor and callback imports
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ROS2 interfaces
from robotiq_3f_gripper_ros2_interfaces.msg import Robotiq3FGripperInputRegisters
from robotiq_3f_gripper_ros2_interfaces.srv import Robotiq3FGripperOutputService

# Others
import time, threading, math
from pymodbus.client import ModbusTcpClient


####### Helper function #######
#                                                                                                                                                      activate | mode   | go-to-pos|emergency| position | speed    |  force
# ros2 service call /Robotiq3FGripper/OutputRegistersService robotiq_3f_gripper_ros2_interfaces/srv/Robotiq3FGripperOutputService "{output_registers: {r_act: 1, r_mod: 1, r_gto: 1, r_atr: 0, r_pra: 255, r_spa: 255, r_fra: 0}}"
#
###############################

class GripperServiceServer(Node):
    '''
    Notes:
    * Starts a service server for controlling the gripper.
    * The service uses the Robotiq3FGripperOutputService.srv custom interface (which uses "Robotiq3FGripperOutputRegisters" msg for interacting with it)
    * Publishes status of gripper to the Robotiq3FGripper/InputRegisters topic (Using "Robotiq3FGripperInputRegisters" msg)
    * Default IP is 172.31.1.69 and the port is 502
    '''
    def __init__(self, gripper_startup_settings:list = [1,1,1,0,255,255,0]):
        '''
        Parameters:
        * gripper_startup_settings: [rACT, rMOD, rGTO, rATR, rPRA, rSPA, rFRA]
        '''
        super().__init__("gripper_control_service_server")
        rclpy.logging.set_logger_level('gripper_control_service_server', rclpy.logging.LoggingSeverity.INFO)
        
        # Declare parameters for node
        self.declare_parameter("gripper_address", "172.31.1.69")
        address = self.get_parameter("gripper_address").get_parameter_value().string_value
        
        self.get_logger().info(f"Gripper service starting on {address}...")
        
        # Variable Init
        self.gripper_connection_init(address)
        self.lock = threading.Lock() # Used for secure comm to gripper?

        # Callback groups
        self.group_1 = MutuallyExclusiveCallbackGroup() # gripper service
        self.group_2 = MutuallyExclusiveCallbackGroup() # gripper read input registers timer
        
        # msgs init
        self.input_registers = Robotiq3FGripperInputRegisters()
        
        # Publisher
        self._input_register_pub = self.create_publisher(Robotiq3FGripperInputRegisters, "Robotiq3FGripper/InputRegisters", 10)

        # Timer
        self._read_input_timer = self.create_timer(0.1, self.read_input_registers, callback_group=self.group_2)
        
        # Gripper init
        #self.gripper_activation(gripper_startup_settings) # TODO: DOESN'T WORK YET
        
        # Service
        self._output_register_service = self.create_service(Robotiq3FGripperOutputService, "Robotiq3FGripper/OutputRegistersService", self.service_callback, callback_group=self.group_1)
        
        self.get_logger().info("Gripper service start-up successful!")



    def gripper_connection_init(self, address):
        self.client = ModbusTcpClient(address) # Create client object
        self.client.connect() # Connect client to gripper
        
        
    def gripper_activation(self, cmd_list: list): # TODO: Add this functionality so the gripper doesn't have to do the activation the first time you use the gripper
        '''
        Activate the gripper by setting starting output values
        
        Parameters:
        cmd_list: [rACT, rMOD, rGTO, rATR, rPRA, rSPA, rFRA]
        '''
        self.get_logger().info("Gripper activating...")

        self.send_data(cmd_list) # Send list to gripper
        
        while True:
            time.sleep(0.2) # Let the gripper process the msg for the given amount of time

            # Assign status to variables
            gSTA = self.input_registers.g_sta
            gIMC = self.input_registers.g_imc
            self.get_logger().info(f"gSTA = {gSTA}    gIMC = {gIMC}")

            # Check the gripper activation, mode, and position has reached stop point
            if gSTA != 0 and gIMC == 3:
                self.get_logger().info("Gripper successfully activated!")
                break
        
        
    
    
    def send_data(self, data):   
        """Send a command to the Gripper - the method takes a list of uint8 as an argument.\n
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
        with self.lock:
            self.client.write_registers(0, message)
    
    
    def output_registers_msg_to_list(self, output_registers_msg):
        '''
        Class function converts given Robotiq3FGripperOutputRegisters msg to a list\n
        List example: [byte_0_int, 0, 0, rPRA, rSPA, rFRA]\n
        byte0: "000{rATR}{rGTO}0{rMOD}{rACT}"
        '''
        
        rACT = output_registers_msg.r_act # Activation
        rMOD = output_registers_msg.r_mod # Mode
        rGTO = output_registers_msg.r_gto # Go to pos
        rATR = output_registers_msg.r_atr # Auto release
        
        byte_0_bits = f"000{rATR}{rGTO}0{rMOD}{rACT}"
        byte_0_int = int(byte_0_bits, base=2)
        
        rPRA = output_registers_msg.r_pra # Target pos
        rSPA = output_registers_msg.r_spa # Grip speed
        rFRA = output_registers_msg.r_fra # Grip force
        
        output_registers_list = [byte_0_int, 0, 0, rPRA, rSPA, rFRA]
        
        return output_registers_list
    
    
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
        
        # Publish input registers to the topic: /Robotiq3FGripper/InputRegisters
        self._input_register_pub.publish(self.input_registers)
    
    
    def service_callback(self, request, response):
        
        output_registers_list = self.output_registers_msg_to_list(request.output_registers) # Convert msg to list
        self.send_data(output_registers_list) # Send list to gripper
        
        
        while True:
            time.sleep(0.2) # Let the gripper process the msg for the given amount of time

            # Assign status to variables
            gSTA = self.input_registers.g_sta
            gIMC = self.input_registers.g_imc
            self.get_logger().info(f"gSTA = {gSTA}    gIMC = {gIMC}")

            # Check the gripper activation, mode, and position has reached stop point
            if gSTA != 0 and gIMC == 3:
                self.get_logger().info("Register message successfully completed")
                response.success = True
                break
        
        return response
    
    
    def shutdown_callback(self):
        self.get_logger().warn("Shutting down...")
        
        
        
        
def main(args=None):
    rclpy.init(args=args)

    # Instansiate node class
    control_service_server_node = GripperServiceServer()

    # Create executor
    executor = MultiThreadedExecutor()
    executor.add_node(control_service_server_node)

    
    try:
        # Run executor
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    
    finally:
        # Shutdown executor
        control_service_server_node.shutdown_callback()
        executor.shutdown()




if __name__ == "__main__":
    main()