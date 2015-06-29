import rospy
import socket
import os
import shutil
import subprocess
import signal
from threading import Lock

class PrismJavaTalker(object):
    
    def __init__(self,port,dir_name,file_name):
        HOST = "localhost"
        PORT = port
        prism_dir = os.path.expanduser("~") + '/bruno-prism/prism-robots/prism/'
        os.chdir(prism_dir)
        os.environ['PRISM_MAINCLASS'] = 'prism.PrismObjectSearch'
        self.java_server=subprocess.Popen(["bin/prism",str(PORT),dir_name, file_name])
        rospy.sleep(1)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(15*60)
        self.sock.connect((HOST, PORT))
        self.directory = dir_name         
        self.lock=Lock()
        

    def get_policy(self,specification):
        command='search\n'
        command=command+specification+'\n'
        self.lock.acquire()
        self.sock.sendall(command)
        data = self.sock.recv(1024)
        self.lock.release()
        #rospy.loginfo("Expected time for search: " +  data)
        return data
    
    
        
    def shutdown(self,remove_dir=True):
        if remove_dir:
            shutil.rmtree(self.directory)        
            rospy.loginfo('prism temp dir removed')
        command='shutdown\n'
        self.lock.acquire()
        self.sock.sendall(command)
        self.sock.close()
        self.lock.release()
        rospy.loginfo("Socket closed")
        os.kill(self.java_server.pid, signal.SIGHUP)
