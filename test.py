
#!/usr/bin/env python
from camera_handler import CameraHandler
import threading
import matplotlib.pyplot as plt
import time
import numpy as np
import os
import cv2

if __name__ == '__main__':
    ch = CameraHandler.getInstance((128,128))#,"828112071102")
        
    # ch.start_pipeline()
    t=threading.Thread(name='display',target=ch.start_pipeline)
    t.start()
    # time needed for camera to warm up to continue getting frames (When running the camera in the background)
    time.sleep(3)

    
    # First confirm the goal position, (manually or use the robot to move the goal to a certain position where can be detected)   "Set up" process
    goal  = ch.get_goal_position()
    print (goal)
    count = 10000
    while (count!=0):
        time.sleep(0.0001)
        goal  = ch.get_goal_position()
        print (goal)
        count = count - 1
    
    
    goal  = ch.get_goal_position()
    while( goal == None ):
        goal  = ch.get_goal_position()


    t3=threading.Thread(name='distance',target=ch.cal_distance_loop)
    t3.start()

    # test average time to get distance
    count = 0
    dis = []
    sumss = 0
    # start = time.time()
    start = time.time()
    
    
    # Define the start position of the moving object (manually or use the robot to move the goal to a certain position where can be detected)
    old = ch.cal_distance()
    while (old ==1):
        old = ch.cal_distance()
    
    while count<1000:
        
        time.sleep(0.005)
        
        # temp = ch.cal_distance() 
        # # take the last value when configuration is not good
        # if (temp == 1):
        #     temp = ch.cal_distance()
        #     temp = ch.cal_distance()
        #     if temp == 1:
        #         temp = old  
        # else:
        #     old = temp
        
        temp = ch.get_distance()
        if (temp == 1):
            temp = old 
        else:
            old = temp

        # print('no')
        dis.append(temp)
        print(temp)
        count=count+1
    end = time.time()
    sumss = end-start-5
    print(sumss/1000)

    # dis = np.array(dis)
    # dis_error = dis[1:]-dis[0:-1]
    # print(dis_error)
    
    
    time_stamp = time.strftime('%Y_%m_%d_%H_%M_%S',time.localtime(time.time()))
    full_path = os.getcwd()
    path_dis = full_path + "/npy/" + time_stamp + "dis.npy"
    dis_data = np.save(path_dis, dis)

    # plt.subplot(1,1,1)
    # plt.subplot(111)
    disdata = np.load(path_dis)
    data_size = len(disdata)
    axis = np.arange( 0, count, 1 )
    lablesize = 18
    fontsize  = 16
    plt.plot(axis, disdata, color = "steelblue", linewidth=1.0, label='distance')
    plt.xlabel('Count',fontsize=lablesize)
    plt.ylabel('Distance[m]',fontsize=lablesize)
    # plt.xticks(fontsize=fontsize)
    # plt.yticks(fontsize=fontsize)
    # plt.legend(loc='lower right',fontsize=18)
    plt.grid(ls='--')
    plt.show()




