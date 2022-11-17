import pyads
from DXL_U2D2_Module import ARM
import argparse
from ctypes import sizeof

CLIENT_IP = "172.16.0.209"
AMSNETID = "172.18.236.91.1.1"

class PLC():
    def __init__(self, arm):
        #PYADS connection started
        self.plc = pyads.Connection(AMSNETID, pyads.PORT_TC3PLC1)
        self.plc.open()
        print(f"Connected ? : {self.plc.is_open}")
        print(f"Local Address ? : {self.plc.get_local_address()}")

        self.arm = arm
        self.handle_dict = {}
        self.handles = None
        self.server_running = False

    def add_handle(self, var_name):
        self.handle_dict[var_name] = self.plc.get_handle(var_name)

    #reads a variable in twincat
    def read(self, variable, type_ = None):
        if type(variable) == 'str':
            print(self.plc.read_by_name(variable))
            return self.plc.read_by_name(variable)
        else:
            print(self.plc.read_by_name("", type_,handle=self.handle_dict[variable]))
            return self.plc.read_by_name("", type_,handle=self.handle_dict[variable])
    
    #write a variable in twincat
    def write(self, variable, data):
        self.plc.write_by_name(variable, data)

    #callback function from the plc server
    def callback(self, notification, data):
        data_type = pyads.PLCTYPE_ARR_INT(4)
        handle, timestamp, value = self.plc.parse_notification(notification, data_type)
        print(handle, timestamp, value)
        if value == [0,0,0,0]:
            self.server_running = False
            return
        
        #move the arm
        self.arm.vecteur(value)
        self.arm.move_arm(1, 20)
        self.arm.wait_stop(1)
        
    #start the plc server
    def listener(self, var):
        self.server_running = True
        attr = pyads.NotificationAttrib(sizeof(pyads.PLCTYPE_ARR_INT(4)))
        self.handles = self.plc.add_device_notification(var, attr, self.callback)
        
    #close the plc communication
    def end(self):
        print(self.handles)
        self.plc.del_device_notification(self.handles[0], self.handles[1])
        for handle in self.handle_dict:
            self.plc.release_handle(self.handle_dict[handle])
        self.plc.close()

        self.arm.disable_torque(arm.arm_list[0])
        self.arm.close_USB_port()



#----------------------Mode server-------------------------
# the arm react each time an array in twincat is modified
if __name__ == "__main__":

    # gets the array in twincat
    parser = argparse.ArgumentParser()
    parser.add_argument("-a","--array", required=True)
    args = vars(parser.parse_args())
    
    #instanciate the arm object 
    arm = ARM(1)

    #runs the plc server
    plc = PLC(arm)
    plc.add_handle(args["array"])
    plc.listener(args["array"])

    #main loop
    try:
        while plc.server_running:
            pass
    except KeyboardInterrupt:
        pass
    
    #close the communication
    plc.end()

    