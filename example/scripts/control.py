#!/usr/bin/env python3

import json
from pynput import keyboard

from RcBrainThread import RcBrainThread
from std_msgs.msg import String, Float32

import rospy

class RemoteControlTransmitterProcess():
    # ===================================== INIT==========================================
    def __init__(self):
        """Run on the PC. It forwards the commans from the user via KeboardListenerThread to the RcBrainThread. 
        The RcBrainThread converts them into actual commands and sends them to the remote via a socket connection.
        
        """
        self.dirKeys   = ['w', 'a', 's', 'd']
        self.paramKeys = ['t','g','y','h','u','j','i','k', 'r', 'p']
        self.pidKeys = ['z','x','v','b','n','m']

        self.allKeys = self.dirKeys + self.paramKeys + self.pidKeys
        
        self.rcBrain   =  RcBrainThread()   
        self.steer_msg = Float32()
        rospy.init_node('EXAMPLEnode', anonymous=False)     
        self.steer_pub = rospy.Publisher('/automobile/steering', Float32, queue_size=3)
        self.publisher = rospy.Publisher('/automobile/command', String, queue_size=1)

    # ===================================== RUN ==========================================
    def run(self):
        """Apply initializing methods and start the threads. 
        """
        with keyboard.Listener(on_press = self.keyPress, on_release = self.keyRelease) as listener: 
            listener.join()
	
    # ===================================== KEY PRESS ====================================
    def keyPress(self,key):
        """Processing the key pressing 
        
        Parameters
        ----------
        key : pynput.keyboard.Key
            The key pressed
        """                                     
        try:                                                
            if key.char in self.allKeys:
                keyMsg = 'p.' + str(key.char)

                self._send_command(keyMsg)
    
        except: pass
        
    # ===================================== KEY RELEASE ==================================
    def keyRelease(self, key):
        """Processing the key realeasing.
        
        Parameters
        ----------
        key : pynput.keyboard.Key
            The key realeased. 
        
        """ 
        if key == keyboard.Key.esc:                        #exit key      
            self.publisher.publish('{"action":"3","steerAngle":0.0}')   
            return False
        try:                                               
            if key.char in self.allKeys:
                keyMsg = 'r.'+str(key.char)

                self._send_command(keyMsg)
    
        except: pass                                                              
                 
    # ===================================== SEND COMMAND =================================
    def _send_command(self, key):
        """Transmite the command to the remotecontrol receiver. 
        
        Parameters
        ----------
        inP : Pipe
            Input pipe. 
        """
        command = self.rcBrain.getMessage(key)
        print("command: ", type(command), command)
        if command is not None:
            original_command = command.copy()
            command = json.dumps(command)
            self.publisher.publish(command)
            # check whether the command, whose type is dict, contains 'steerAngle'
            if 'steerAngle' in original_command:
                print("found, ",original_command['steerAngle'])
                self.steer_msg.data = original_command['steerAngle']
                self.steer_pub.publish(self.steer_msg)
            else: 
                print("not found")
                self.steer_msg.data = 0.0
                self.steer_pub.publish(self.steer_msg)
        else:
            self.steer_msg.data = 0.0
            self.steer_pub.publish(self.steer_msg)
        # print("steer: ", self.steer_msg.data)

            
if __name__ == '__main__':
    try:
        nod = RemoteControlTransmitterProcess()
        nod.run()
    except rospy.ROSInterruptException:
        pass