# Basic ROS2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# Executor and callback imports
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ROS2 interfaces
from robotiq_3f_gripper_ros2_interfaces.msg import Robotiq3FGripperInputRegisters, Robotiq3FGripperOutputRegisters
from robotiq_3f_gripper_ros2_interfaces.action import Robotiq3FGripperOutputGoal


# Others
import time, threading, math
from pymodbus.client import ModbusTcpClient




# ros2 action send_goal -f /gripper_position robotiq_3f_gripper_ros2_interfaces/action/Robotiq3FGripperOutputGoal "{output_registers_goal: {r_act: 1, r_mod: 1, r_gto: 1, r_atr: 0, r_pra: 0, r_spa: 255, r_fra: 0}}"

class GripperActionServer(Node):
    '''
    Notes:
    * Starts an action server for controlling the gripper.
    * The service uses the Robotiq3FGripperOutputGoal.action custom interface (which uses "Robotiq3FGripperOutputRegisters" msg for interacting with it)
    '''
    def __init__(self):
        super().__init__("gripper_control_action_server")
        rclpy.logging.set_logger_level('gripper_control_action_server', rclpy.logging.LoggingSeverity.INFO)
        
        # Declare parameters for node
        self.declare_parameter("gripper_address", "172.31.1.69")
        address = self.get_parameter("gripper_address").get_parameter_value().string_value
        
        self.get_logger().info(f"Gripper service starting on {address}...")
        
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
        
        # Actions
        self._gripper_action_server = ActionServer(self, Robotiq3FGripperOutputGoal, "gripper_position", self.action_server_callback, callback_group=self.group_3)
        
        self.get_logger().info("Gripper action start-up successful!")



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
        Class function for reading input registers from the gripper and returning the Robotiq3FGripperInputRegisters message type
        '''
        
        numRegs = int(math.ceil(numBytes/2))

        with self.lock:
            response = self.client.read_input_registers(0,numBytes)
        
        response_byte_array = []

        for i in range(0, numRegs): # Splits the 16 bit registers from pymodbus into 2x 8 bit registers (uint8) 
            response_byte_array.append((response.getRegister(i) & 0xFF00) >> 8)
            response_byte_array.append(response.getRegister(i) & 0x00FF)
            
        
        # Create input register msg
        input_registers_msg = Robotiq3FGripperInputRegisters()
        
        # Setting Gripper status register
        response_byte0_bit_string = format(response_byte_array[0], '08b') # Format int to 8 bits
        
        input_registers_msg.g_act = int(response_byte0_bit_string[7],   base=2) # gACT ->       Initialization status       -> 1 = Gripper activation
        input_registers_msg.g_mod = int(response_byte0_bit_string[5:7], base=2) # gMOD ->       Operation mode status       -> 11 = Done and ready
        input_registers_msg.g_gto = int(response_byte0_bit_string[4],   base=2) # gGTO -> Activation and mode change status -> 11 = Done and ready
        input_registers_msg.g_imc = int(response_byte0_bit_string[2:4], base=2) # gIMC -> Activation and mode change status -> 11 = Done and ready
        input_registers_msg.g_sta = int(response_byte0_bit_string[0:2], base=2) # gSTA ->           Motion status           -> 11 = Completed successfully
        
        # Setting Fault status register
        input_registers_msg.g_flt = response_byte_array[2]
        
        # Setting Position request echo register
        input_registers_msg.g_pra = response_byte_array[3]
        
        return input_registers_msg
    
    
    def action_server_callback(self, goal_handle):
        
        # Initiate variables used for the action server
        request_msg = goal_handle.request.output_registers_goal
        feedback_msg = Robotiq3FGripperOutputGoal.Feedback()
        result_msg = Robotiq3FGripperOutputGoal.Result()

        # Convert request msg to list and then send command to gripper
        request_msg_list = self.output_registers_msg_to_list(request_msg) # Convert request msg to list
        self.get_logger().info(f"Sending following output register list: {request_msg_list}")
        self.send_data(request_msg_list) # Send request msg list to gripper

        while True:
            
            time.sleep(0.2) # Let the gripper process the msg for the given amount of time

            
            gripper_input_registers = self.read_input_registers() # Read the status of the gripper

            # Assign the used status variables 
            gSTA = gripper_input_registers.g_sta
            gIMC = gripper_input_registers.g_imc
            self.get_logger().info(f"gSTA = {gSTA}    gIMC = {gIMC}")
            
            
            # Assign the action msgs
            feedback_msg.input_registers = gripper_input_registers
            feedback_msg.output_registers = request_msg
            goal_handle.publish_feedback(feedback_msg)

            # Check the gripper activation, mode, and position has reached stop point
            if gSTA != 0 and gIMC == 3:
                self.get_logger().info("Register message successfully completed")
                break
        
        
        # Set action as successfully completed
        result_msg.result = 'Success'
        self.get_logger().info(f"Result: {result_msg.result}")
        
        goal_handle.succeed()
        
        return result_msg
    
    
    def shutdown_callback(self):
        self.get_logger().warn("Shutting down...")
        
        
        
        
def main(args=None):
    rclpy.init(args=args)

    # Instansiate node class
    control_action_server_node = GripperActionServer()

    # Create executor
    executor = MultiThreadedExecutor()
    executor.add_node(control_action_server_node)

    
    try:
        # Run executor
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    
    finally:
        # Shutdown executor
        control_action_server_node.shutdown_callback()
        executor.shutdown()




if __name__ == "__main__":
    main()