from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import msvcrt                                  # Keyboard input library
import numpy as np
import time


# Control table address
ADDR_MX_TORQUE_ENABLE      = 24                # Control table address is different for each dynamixel model => go look at the data sheet
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_MOVING_SPEED       = 32                # Dynamixel moving speed in joint mode
ADDR_MX_MOVING_STATUS      = 46                # bit to 1 when the motor is moving
ADDR_MX_PRESENT_LOAD       = 40

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel => AX-12A work only with protocol 1.0

# Default setting
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = "COM4"            # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0                 # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 1                 # Dynamixel moving status threshold 
OPERATION_MODE              = 0              # Mode is unavailable in Protocol 1.0 Reset

# robotic arm constants
AXIS_NUMBER                 = 4                 # degree of liberty of the arm
SHOULDER_LENGTH             = 97                # distance between each axis of the arm in milimeter
ELBOW_LENGTH                = 97
WRIST_LENGTH                = 0
BASE_DECENTERING            = 35                # distance between the shoulder axis and the base axis


class DXL:
    def __init__(self):
        # package status
        self.dxl_comm_result = None
        self.dxl_error = None
        self.dxl_present_position = None
        self.dxl_present_load = None

        # Open USB port
        self.portHandler = PortHandler(DEVICENAME)
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            msvcrt.getch()
            quit()
        
        # initialize dynamixel protocole
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            msvcrt.getch()
            quit()
    

    def enable_torque(self,id_list):
        # Enable Dynamixel Torque
        for ID in id_list:
            self.dxl_comm_result, self.dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            if self.dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
            elif self.dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))
            else:
                print(f"Dynamixel {ID} has been successfully connected")


    def disable_torque(self,DXL_ID_list):
        # Disable Dynamixel Torque
        for ID in DXL_ID_list:
            self.dxl_comm_result, self.dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            if self.dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
            elif self.dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))


    def close_USB_port(self):
        # Close port
        self.portHandler.closePort()

    def degree2Byte(self, num):
        #convert degree (0..300) value to analog (2 Bytes => 0...1023)
        return num*1023/300

    def Byte2Degree(self, num):
        #convert analog (2 Bytes => 0...1023) to degree value (0...300)
        return num*300/1023

    def servo(self, ID, position, speed):
        # Write moving speed
        self.dxl_comm_result, self.dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, ADDR_MX_MOVING_SPEED, speed)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))

        # Write goal position
        self.dxl_comm_result, self.dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, ADDR_MX_GOAL_POSITION, position)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))


    def get_position(self, ID):
        # Read present position
        self.dxl_present_position, self.dxl_comm_result, self.dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, ADDR_MX_PRESENT_POSITION)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))
        return self.dxl_present_position


    def get_position_periodic(self,ID,dxl_goal_position):
        while 1:
            # Read present position till the goal position is obtained
            self.dxl_present_position, self.dxl_comm_result, self.dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, ADDR_MX_PRESENT_POSITION)
            if self.dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
            elif self.dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))

            #print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position, self.dxl_present_position))
            # a threshold is set for the goal position
            if not abs(dxl_goal_position - self.dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                break
        return True

    
    def get_load(self, ID):
        # Read present load
        self.dxl_present_load, self.dxl_comm_result, self.dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, ADDR_MX_PRESENT_LOAD)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))
        return self.dxl_present_load


class ARM(DXL):
    #Arm manipulator
    def __init__(self, number_of_arm):
        self.number_of_joint = number_of_arm * AXIS_NUMBER
        #create an ID list for each arm
        #example: 2 arms => [[1,2,3,4],[5,6,7,8]]]
        self.arm_list = [[1+self.number_of_joint - ((self.number_of_joint - x*AXIS_NUMBER) - y) for y in range(0,AXIS_NUMBER)] for x in range(0, number_of_arm)]    
        # access parent class constructor
        super().__init__()

        #enable torque
        for arm in self.arm_list:
            self.enable_torque(arm)


    def vecteur(self,coord):
        # convert cartesian coordinates [x,y,z,approach angle] to angles for each axis of the arm 
        # if further help for comprehension is needed => contact Alec.Horth@Bobst.com
        self.x = coord[0]
        self.y = coord[1]
        self.z = coord[2]
        self.approach_angle = 90 - coord[3]
        self.base = np.degrees(np.arctan(self.z/self.x))
        self.relative_x = np.hypot(self.x,self.z) 
        self.hyp = np.hypot(self.relative_x,self.y)
        self.hyp_angle = np.degrees(np.arctan(self.y/self.relative_x))
        # shoulder angle => formula of the crossing of two circles
        self.shoulder = np.degrees(np.arccos((np.power(ELBOW_LENGTH,2) - (np.power(self.hyp, 2) + np.power(SHOULDER_LENGTH,2)))/(-2*SHOULDER_LENGTH*self.hyp)))
        self.elbow = 180 - 2*self.shoulder


    def __get_shoulder(self):
        return int((self.shoulder + self.hyp_angle + 150) * (1023/300))


    def __get_elbow(self):
        return int((self.elbow - 30) * (1023/300))


    def __get_wrist(self):
        return int((60 - self.hyp_angle + self.shoulder + self.approach_angle) * (1023/300))


    def __get_base(self):
        return int((self.base + 150)* (1023/300))

    
    #speed between 0 and 100
    def move_arm(self, arm, speed):
        speed = int(speed * 1023/100)
        self.servo(self.arm_list[arm - 1][3],self.__get_wrist(),speed-70)
        self.servo(self.arm_list[arm - 1][2],self.__get_elbow(),speed)   
        self.servo(self.arm_list[arm - 1][1],self.__get_shoulder(),speed)
        self.servo(self.arm_list[arm - 1][0],self.__get_base(),speed)

    
    #go to a constant preseted position
    def reset_position(self, arm):
        speed = int(15 * 1023/100)
        self.servo(self.arm_list[arm - 1][3],220,speed)
        self.servo(self.arm_list[arm - 1][2],22,speed)   
        self.servo(self.arm_list[arm - 1][1],1012,speed)
        self.servo(self.arm_list[arm - 1][0],517,speed)

    
    def __plus_ou_moins(self, valeur1,valeur2, threshold):
        return valeur1 < valeur2 + threshold and valeur1 > valeur2 - threshold


    # wait till all of the joints get to their position
    def wait_position(self, arm, timeout = 4):
        counter = 0
        time_stamp = time.time()
        kinematicchain = [self.get_base(), self.get_shoulder(), self.get_elbow(), self.get_wrist()]
        
        while time.time() - time_stamp < timeout:
            #print(self.get_position(self.arm_list[arm - 1][counter]))
            #print(kinematicchain[counter])
            if self.__plus_ou_moins(self.get_position(self.arm_list[arm - 1][counter]), kinematicchain[counter],6):
                counter += 1
            if counter > 3:
                break
    

    # wait till all of the joint stop moving
    def wait_stop(self, arm, timeout = 4):
        counter = 0
        time_stamp = time.time()
        while time.time() - time_stamp < timeout:
            data, _, _ = self.packetHandler.readTxRx(self.portHandler, self.arm_list[arm - 1][counter], ADDR_MX_MOVING_STATUS, 1)
            if data[0] == 0:
                counter += 1
            if counter > 3:
                break
            



#--------------Demo Program--------------------
if __name__ == "__main__":
    index = 0
    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE+700, DXL_MAXIMUM_POSITION_VALUE-100]         # Goal position
    dxl_speed = [300, 1023]
    id_list = [4]
    dxl = DXL()
    dxl.enable_torque(id_list)
    
    while 1:
        print("Press any key to continue! (or press ESC to quit!)")
        if msvcrt.getch() == chr(27).encode():
            break
        dxl.servo(id_list[0],dxl_goal_position[index],dxl_speed[index])
        #dxl.get_position_periodic(DXL_ID,dxl_goal_position[index])

        # Change goal position
        if index == 0:
            index = 1
        else:
            index = 0


    # Disable Dynamixel Torque
    dxl.disable_torque(id_list)
    # Close port
    dxl.close_USB_port()