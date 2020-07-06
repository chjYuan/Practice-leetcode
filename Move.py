import socket
import time
import os
import rtde_control
import rtde_receive
import numpy as np
import sys
import time
import datetime
import matplotlib.pyplot as plt
import threading
from numpy import linalg as LA
import transforms3d


class Move_URRobot():
    def __init__(self, ur_ipaddress=None, speed_ms=0.25, accel_mss=0.5):
        if ur_ipaddress == None:
            self.ur_ipaddress = "10.3.15.240"
        else:
            self.ur_ipaddress = ur_ipaddress

        self.urrobot_r = rtde_receive.RTDEReceiveInterface(self.ur_ipaddress)
        self.r_control = rtde_control.RTDEControlInterface(self.ur_ipaddress)
        self.HOST = self.ur_ipaddress  # The UR IP address
        self.PORT = 30003  # UR secondary client

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))

        self.speed_ms = speed_ms
        self.accel_mss = accel_mss
        self.path = []

    def Sinpath(self, TCP_pose_ori, Amp, period, step, blend, type_fix):
        num = step
        T = period
        A = Amp
        blend_radius = blend
        path = []
        target_pose = []
        # Same height with a certian Z value
        if type_fix == 'z':
            for i in range(num):
                x = TCP_pose_ori[0] + i * period / (num - 1)
                y = TCP_pose_ori[1] + A * np.sin(
                    (2 * np.pi / T) * (i * period / (num - 1)))
                path.append([
                    x, y, TCP_pose_ori[2], TCP_pose_ori[3], TCP_pose_ori[4],
                    TCP_pose_ori[5], self.speed_ms, self.accel_mss,
                    blend_radius
                ])

                target_pose.append([x, y, TCP_pose_ori[2]])

        if type_fix == 'x':
            for i in range(num):
                y = TCP_pose_ori[1] + i * period / (num - 1)
                z = TCP_pose_ori[2] + A * np.sin(
                    (2 * np.pi / T) * (i * period / (num - 1)))
                path.append([
                    TCP_pose_ori[0], y, z, TCP_pose_ori[3], TCP_pose_ori[4],
                    TCP_pose_ori[5], self.speed_ms, self.accel_mss,
                    blend_radius
                ])

                target_pose.append([TCP_pose_ori[0], y, z])

        if type_fix == 'y':
            for i in range(num):
                x = TCP_pose_ori[0] + i * period / (num - 1)
                z = TCP_pose_ori[2] + A * np.sin(
                    (2 * np.pi / T) * (i * period / (num - 1)))
                path.append([
                    x, TCP_pose_ori[1], z, TCP_pose_ori[3], TCP_pose_ori[4],
                    TCP_pose_ori[5], self.speed_ms, self.accel_mss,
                    blend_radius
                ])

                target_pose.append([x, TCP_pose_ori[1], z])
        # self.r_control.moveL(path)

        return path, target_pose

    def Move_circle(self, radius):

        speed_ms = self.speed_ms
        accel_mss = self.accel_mss
        blend = 0.005

        circle_radius = radius
        circle_point = 100
        path = []

        ActualTCPPose = self.urrobot_r.getActualTCPPose()
        Rvec = [ActualTCPPose[3], ActualTCPPose[4], ActualTCPPose[5]]
        axis_angle = LA.norm(Rvec)

        R_matrix = transforms3d.axangles.axangle2mat([
            ActualTCPPose[3] / axis_angle, ActualTCPPose[4] / axis_angle,
            ActualTCPPose[5]/ axis_angle
        ] , axis_angle)

        for i in range(circle_point + 2):

            pos_wrt_EE_x = circle_radius * np.cos(i * 2 * np.pi / circle_point)
            pos_wrt_EE_y = circle_radius * np.sin(i * 2 * np.pi / circle_point)
            pos_wrt_EE = np.array([pos_wrt_EE_x, pos_wrt_EE_y, 0])

            T_wrt_base = np.array(
                [ActualTCPPose[0], ActualTCPPose[1], ActualTCPPose[2]])

            pos_wrt_base = R_matrix.dot(pos_wrt_EE) + T_wrt_base

            waypoint = [
                pos_wrt_base[0], pos_wrt_base[1], pos_wrt_base[2],
                ActualTCPPose[3], ActualTCPPose[4], ActualTCPPose[5], speed_ms,
                accel_mss, blend
            ]

            path.append(waypoint)

        self.r_control.moveL(path)
        self.path = path
        # print(path)

    def Move_backandforth(self, point_ori, length, count):

        speed_ms = self.speed_ms
        accel_mss = self.accel_mss
        blend = 0.000

        ActualTCPPose = self.urrobot_r.getActualTCPPose()

        Point_start = np.array(
            [ActualTCPPose[0], ActualTCPPose[1], ActualTCPPose[2]])
        Point_end = np.array([point_ori[0], point_ori[1], point_ori[2]])

        direction_vec = Point_end - Point_start
        direction_vec_norm = np.linalg.norm(direction_vec)

        target1 = Point_start + length * (direction_vec / direction_vec_norm)
        target2 = Point_start - length * (direction_vec / direction_vec_norm)

        path = []

        for i in range(count):
            waypoint = [
                target1[0], target1[1], target1[2], ActualTCPPose[3],
                ActualTCPPose[4], ActualTCPPose[5], speed_ms, accel_mss, blend
            ]
            path.append(waypoint)
            waypoint = [
                target2[0], target2[1], target2[2], ActualTCPPose[3],
                ActualTCPPose[4], ActualTCPPose[5], speed_ms, accel_mss, blend
            ]
            path.append(waypoint)

        waypoint_end = [
            ActualTCPPose[0], ActualTCPPose[1], ActualTCPPose[2],
            ActualTCPPose[3], ActualTCPPose[4], ActualTCPPose[5], speed_ms,
            accel_mss, blend
        ]
        path.append(waypoint_end)

        self.r_control.moveL(path)
        # print(path)

    def Move_cone_aboveskin(self, radius, depth):
        speed_ms = self.speed_ms
        accel_mss = self.accel_mss
        blend = 0.005

        circle_radius = radius
        cone_depth = depth
        circle_point = 4
        path = []

        ActualTCPPose = self.urrobot_r.getActualTCPPose()
        Rvec = [ActualTCPPose[3], ActualTCPPose[4], ActualTCPPose[5]]
        axis_angle = LA.norm(Rvec)

        R_matrix_base_EE = transforms3d.axangles.axangle2mat([
            ActualTCPPose[3] / axis_angle, ActualTCPPose[4] / axis_angle,
            ActualTCPPose[5]/ axis_angle
        ] , axis_angle)

        for i in range(circle_point):
            pos_wrt_EE_x = circle_radius * np.cos(i * 2 * np.pi / circle_point)
            pos_wrt_EE_y = circle_radius * np.sin(i * 2 * np.pi / circle_point)

            # points on the circle (EE frame)
            pos_wrt_EE = np.array([pos_wrt_EE_x, pos_wrt_EE_y, -cone_depth])

            # center of the circle of the cone(EE frame)
            poscenter_wrt_EE = np.array([0, 0, -cone_depth])

            # vertex of the cone (EE frame)
            vertex_wrt_EE = np.array([0, 0, 0])

            # vector circlepoint->center
            vec_circlepoint_center = poscenter_wrt_EE - pos_wrt_EE

            # vector circlepoint->vertex
            vec_circlepoint_vertex = vertex_wrt_EE - pos_wrt_EE

            # R matrix with respect to the EE frame
            vec_y_EE = np.cross(vec_circlepoint_vertex, vec_circlepoint_center)
            vec_y_EE_norm = LA.norm(vec_y_EE)

            vec_z_EE = vec_circlepoint_vertex
            vec_z_EE_norm = LA.norm(vec_z_EE)

            vec_x_EE = np.cross(vec_y_EE, vec_z_EE)
            vec_x_EE_norm = LA.norm(vec_x_EE)

            R_point_EE = np.array([(vec_x_EE / vec_x_EE_norm),
                                   (vec_y_EE / vec_y_EE_norm),
                                   (vec_z_EE / vec_z_EE_norm)])
            R_point_EE = R_point_EE.T

            # R matrix with respect to the base
            R_point_base = R_matrix_base_EE.dot(R_point_EE)

            direc, angle = transforms3d.axangles.mat2axangle(R_point_base)
            rx = direc[0] * angle
            ry = direc[1] * angle
            rz = direc[2] * angle

            waypoint = [
                ActualTCPPose[0], ActualTCPPose[1], ActualTCPPose[2], rx, ry,
                rz, speed_ms, accel_mss, blend
            ]
            path.append(waypoint)

        self.r_control.moveL(path)

        print(path)

    def Move_cone_underskin(self, radius, depth):
        speed_ms = self.speed_ms
        accel_mss = self.accel_mss
        blend = 0.005

        circle_radius = radius
        cone_depth = depth
        circle_point = 50
        path = []

        ActualTCPPose = self.urrobot_r.getActualTCPPose()
        Rvec = [ActualTCPPose[3], ActualTCPPose[4], ActualTCPPose[5]]
        axis_angle = LA.norm(Rvec)

        R_matrix_base_EE = transforms3d.axangles.axangle2mat([
            ActualTCPPose[3] / axis_angle, ActualTCPPose[4] / axis_angle,
            ActualTCPPose[5] / axis_angle] , axis_angle)

        for i in range(circle_point):
            pos_wrt_EE_x = circle_radius * np.cos(i * 2 * np.pi / circle_point)
            pos_wrt_EE_y = circle_radius * np.sin(i * 2 * np.pi / circle_point)

            # points on the circle (EE frame)
            pos_wrt_EE = np.array([pos_wrt_EE_x, pos_wrt_EE_y, 0])

            # center of the circle of the cone(EE frame)
            poscenter_wrt_EE = np.array([0, 0, 0])

            # vertex of the cone (EE frame)
            vertex_wrt_EE = np.array([0, 0, cone_depth])

            # vector circlepoint->center
            vec_circlepoint_center = poscenter_wrt_EE - pos_wrt_EE

            # vector circlepoint->vertex
            vec_circlepoint_vertex = vertex_wrt_EE - pos_wrt_EE

            # direction fix y , calculate x first, recaculate y
            vec_z_EE = vec_circlepoint_vertex
            vec_z_EE_norm = LA.norm(vec_z_EE)

            #
            temp = np.array([circle_radius, 0, 0])
            # vec_y_EE = poscenter_wrt_EE - temp
            vec_y_EE = R_matrix_base_EE[:,1]
            vec_x_EE = np.cross(vec_y_EE, vec_z_EE)
            vec_x_EE_norm = LA.norm(vec_x_EE)

            vec_y_EE = np.cross(vec_z_EE, vec_x_EE)
            vec_y_EE_norm = LA.norm(vec_y_EE)


            # R matrix with respect to the EE frame  /Last joint rotates all the time
            # vec_y_EE = np.cross(vec_circlepoint_vertex,vec_circlepoint_center)
            # vec_y_EE_norm = LA.norm(vec_y_EE)

            # vec_z_EE = vec_circlepoint_vertex
            # vec_z_EE_norm = LA.norm(vec_z_EE)

            # vec_x_EE = np.cross(vec_y_EE,vec_z_EE)
            # vec_x_EE_norm = LA.norm(vec_x_EE)

            R_point_EE = np.array([(vec_x_EE / vec_x_EE_norm),
                                   (vec_y_EE / vec_y_EE_norm),
                                   (vec_z_EE / vec_z_EE_norm)])
            R_point_EE = R_point_EE.T

            # R matrix with respect to the base
            R_point_base = R_matrix_base_EE.dot(R_point_EE)

            direc, angle = transforms3d.axangles.mat2axangle(R_point_base)
            rx = direc[0] * angle
            ry = direc[1] * angle
            rz = direc[2] * angle

            # new circle
            T_wrt_base = np.array(
                [ActualTCPPose[0], ActualTCPPose[1], ActualTCPPose[2]])
            pos_wrt_base = R_matrix_base_EE.dot(pos_wrt_EE) + T_wrt_base

            waypoint = [
                pos_wrt_base[0], pos_wrt_base[1], pos_wrt_base[2], rx, ry, rz,
                speed_ms, accel_mss, blend
            ]
            path.append(waypoint)

        self.r_control.moveL(path)
        self.path = path
        print(path)

    def move_duplicate(self, num=2):

        path = []
        for i in range(num):
            path.append(self.path)

        self.r_control.moveL(path)

    def plotforce(self, num=80000):
        data_num = num
        FTdata = []
        for x in range(data_num):
            FT = self.urrobot_r.getActualTCPForce()
            FTdata.append(FT)

        # timestamp = np.append(timestamp, time.time())
        time_stamp = time.strftime('%Y_%m_%d_%H_%M_%S',
                                   time.localtime(time.time()))
        full_path = os.getcwd()
        path_force = full_path + "/npy/" + time_stamp + "force.npy"
        force_data = np.save(path_force, FTdata)

        ext_force = np.load(path_force)
        data_size = len(ext_force[:, 0])

        axis = np.arange(1, data_size + 1, 1)
        lablesize = 18
        fontsize = 16
        plt.plot(axis,
                 ext_force[:, 0],
                 color="steelblue",
                 linewidth=1.0,
                 label='Fx_ext')
        plt.plot(axis,
                 ext_force[:, 1],
                 color="darkorange",
                 linewidth=1.0,
                 label='Fy_ext')
        plt.plot(axis,
                 ext_force[:, 2],
                 color="forestgreen",
                 linewidth=1.0,
                 label='Fz_ext')
        plt.xlabel('Time[ms]', fontsize=lablesize)
        plt.ylabel('Force[N]', fontsize=lablesize)
        plt.xticks(fontsize=fontsize)
        plt.yticks(fontsize=fontsize)
        plt.legend(loc='lower right', fontsize=18)
        plt.grid(ls='--')
        plt.show()

    def plotvel(self, num=80000):
        data_num = num
        VELdata = []
        for x in range(data_num):
            VEL = self.urrobot_r.getActualTCPSpeed()
            VELdata.append(VEL)

        time_stamp = time.strftime('%Y_%m_%d_%H_%M_%S',
                                   time.localtime(time.time()))
        full_path = os.getcwd()
        path_vel = full_path + "/npy/" + time_stamp + "vel.npy"
        vel_data = np.save(path_vel, VELdata)

        tcp_vel = np.load(path_vel)
        data_size = len(tcp_vel[:, 0])

        axis = np.arange(1, data_size + 1, 1)
        lablesize = 18
        fontsize = 16
        plt.plot(axis,
                 tcp_vel[:, 0],
                 color="steelblue",
                 linewidth=1.0,
                 label='Vx_ext')
        plt.plot(axis,
                 tcp_vel[:, 1],
                 color="darkorange",
                 linewidth=1.0,
                 label='Vy_ext')
        plt.plot(axis,
                 tcp_vel[:, 2],
                 color="forestgreen",
                 linewidth=1.0,
                 label='Vz_ext')
        plt.xlabel('Time[ms]', fontsize=lablesize)
        plt.ylabel('VEL_TCP[N]', fontsize=lablesize)
        plt.xticks(fontsize=fontsize)
        plt.yticks(fontsize=fontsize)
        plt.legend(loc='lower right', fontsize=18)
        plt.grid(ls='--')
        plt.show()

    def plotpose(self, num=10000):
        data_num = num
        actual_pose = []
        for x in range(data_num):
            time.sleep(0.003)
            # VEL  = self.urrobot_r.getActualTCPPose()
            actual_pose.append(self.urrobot_r.getActualTCPPose())

        time_stamp = time.strftime('%Y_%m_%d_%H_%M_%S',
                                   time.localtime(time.time()))
        full_path = os.getcwd()
        path_poseactual = full_path + "/npy/" + time_stamp + "pose_actual.npy"
        np.save(path_poseactual, actual_pose)

        tcp_pose_actual = np.load(path_poseactual)
        data_size = len(tcp_pose_actual[:, 0])

        axis = tcp_pose_actual[:, 0]

        lablesize = 18
        fontsize = 16
        plt.plot(axis,
                 tcp_pose_actual[:, 1],
                 color="steelblue",
                 linewidth=1.0,
                 label='Vx_ext')
        plt.xlabel('x', fontsize=lablesize)
        plt.ylabel('y', fontsize=lablesize)
        plt.xticks(fontsize=fontsize)
        plt.yticks(fontsize=fontsize)
        plt.legend(loc='lower right', fontsize=18)
        plt.grid(ls='--')
        plt.show()
