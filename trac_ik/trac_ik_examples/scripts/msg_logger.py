#!/usr/bin/env python3

from tqdm import tqdm
import rospy
import rosparam

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#msg
from geometry_msgs.msg import WrenchStamped


class msg_logger():
    def __init__(self):
        #subscriber
        self.subscriber = rospy.Subscriber("/ft_sensor/raw",WrenchStamped,self.callback)
        #menber argments
        self.force_x = []
        self.force_y = []
        self.force_z = []
        self.time_stamp = []
        self.save_location = "./"
        self.save_name= "temp"

    def callback(self,data):

        self.time_stamp.append(data.header.seq)
        self.force_x.append(data.wrench.force.x)
        self.force_y.append(data.wrench.force.y)
        self.force_z.append(data.wrench.force.z)

    def save_npy(self,save_path,save_name):
        self.save_location = save_path
        self.save_name= save_name
        #np.savez(self.save_location+save_name+".npz", time_stamp = self.time_stamp, force_x = self.force_x,force_y = self.force_y,force_z = self.force_z)
        print("saved as"+self.save_location)

        self.gen_animation()

    
    def gen_animation(self):
        npz_comp = np.load(self.save_location+self.save_name+".npz")
        print(npz_comp.files)
        time_stamp = npz_comp[npz_comp.files[0]]
        print("data_size",len(time_stamp))
        f_x =npz_comp[npz_comp.files[1]]
        f_y =npz_comp[npz_comp.files[2]]
        f_z =npz_comp[npz_comp.files[3]]
        
        plt.plot(time_stamp, f_x,linestyle="solid")
        plt.savefig(self.save_location+self.save_name+".pdf")
        
        # fig = plt.figure()

        # def update(i):
        #     if i != 0:
        #         plt.cla()
        #     plt.plot(time_stamp[i],f_x[i],'r')

        # ani = animation.FuncAnimation(fig,update,interval=100,frames=len(time_stamp))

        # ani.save('animation_test.mp4', writer="ffmpeg",fps=30)
        print("done!!")

def main():
    rospy.init_node("data_logger",anonymous=True)
    data_logger=msg_logger()
    # spin
    rospy.spin()
    # ratesleep
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        pub.send_msg()
        rate.sleep()
    if(rospy.is_shutdown()==True):
        data_logger.save_npy(save_path ="/home/haxhi/catkin_second/src/iksolver_track/trac_ik/ft_sensor_data/",save_name="test")




if __name__ == '__main__':
    main()