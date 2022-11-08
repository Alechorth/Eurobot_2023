from DXL_U2D2_Module import ARM
from DXL_PYADS_Module import PLC
import pyads
import argparse


#get argument from Twincat3 : NT_StartProcess 
parser = argparse.ArgumentParser()
parser.add_argument("-a","--array", required=True)
args = vars(parser.parse_args())

#create the arm and plc objects
arm = ARM(1)
plc = PLC(arm)


#add the twincat3 variable to the handles manager
plc.add_handle(args["array"])

#the arm go to various position
arm.vecteur([100,-30,50,83])
arm.move_arm(1, 20) 
arm.wait_stop(1)
arm.vecteur([100,100,-50,83])
arm.move_arm(1, 20)
arm.wait_stop(1)

#the arm reads à positin in Twincat3
arm.vecteur(plc.read(args["array"],pyads.PLCTYPE_ARR_INT(4)))
arm.move_arm(1, 20)
arm.wait_stop(1)

#start a plc server that launches à callback function each time the variable changes state
plc.listener(args["array"])

#the arm resets it's position
arm.reset_position(1)
arm.wait_stop(1)

#close all the communications
arm.disable_torque(arm.arm_list[0])
arm.close_USB_port()
plc.end()
print("done")