# -*- coding: utf-8 -*-
from threading import Thread
import time
from tkinter import *
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
from dobot_api import *
import json
from files.alarm_controller import alarm_controller_list
from files.alarm_servo import alarm_servo_list
import time
from ultralytics import YOLO
# 라이브러리 불러오기
import threading
import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
from time import sleep
from pose_ArUCo.pose_estimation import run_aruco
from pose_ArUCo.detect_aruco_images import detect_marker
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from rotate import transform_cam_to_robot
import ham_api

ham_robotui = ham_api.RobotUI()

color_list = []

def enable_robot(robot_api):
    robot_api.connect_port()
    robot_api.client_dash.ClearError()
    robot_api.enable()

def disable_robot(robot_api):
    ham_robotui.enable()
    ham_robotui.client_dash.ClearError()
    ham_robotui.connect_port()

def recovery_robot(robot_api):
    ham_robotui.enable()
    ham_robotui.client_dash.ClearError()
    ham_robotui.enable()

def first_setting(robot_api):
    # gripper off
    robot_api.client_dash.ToolDO(1, 0)
    # GET current_pose
    current_pose = robot_api.client_dash.GetPose().split("{")[1].split("}")[0].split(",")
    current_pose = list(map(float, current_pose))
    # print current_pose (for debug)
    # print("--------------")
    # print("current_pose")
    # print(current_pose)
    # print("--------------")
    if current_pose[2] < -60 and current_pose[2] > 70:
        current_pose[2]=0
    # move safety_position
    robot_api.client_move.MovJ(current_pose[0],current_pose[1],current_pose[2],current_pose[3])
    # move home_position
    move_HOME_position(ham_robotui, current_pose)

def ham_aruco1(edit_x = -35.4, edit_y = -25.9, edit_z = -130.4, edit_yaw = 0, img = None):
    point_grip_dic = {}
    # 마커 검출
    detected_markers = detect_marker(image=img, marker_type="DICT_4X4_100")
    # 아르코 마커로 카메라 좌표계 기준 6D pose 예측하기
    # rvec : 회전 행렬 (roll, pitch, yaw)   // unit : rads
    # tvec : 이동 행렬 (x, y, z)            // unit : meter
    # corners : 아르코 마커의 각 코너 좌표  // (x, y) 픽셀 위치
    rvec, tvec, ids_list ,corners = run_aruco(img)
    # print(ids_list)
    # print(len(rvec))
    # print(len(tvec))
    # print(len(corners))
    for i in range(len(rvec)) :
        # 간단한 전처리
        rvec_tmp = np.array(rvec[i]).flatten()
        tvec_tmp = np.array(tvec[i]).flatten()
        corners_tmp = np.array(corners[i][0][0])
        # 회전 행렬, 이동 행렬, 마커 코너 좌표 출력
        # print(f"rotate vector : {rvec_tmp} rads")
        # print(f"translation vector : {tvec_tmp} meters \n")
        # print(f"marker corners : {corners_tmp}")
        # 오차 보정을 통한 좌표 및 각도 보정
        tvec_tmp[0] = round(tvec_tmp[0], 4)
        tvec_tmp[1] = round(tvec_tmp[1], 4)
        tvec_tmp[2] = round(tvec_tmp[2], 4)
        yaw_angle_by_cam = round(rvec_tmp[-1], 4)
        #print('rvec : ', rvec_tmp)
        # print(f"x based camera cordinate: {tvec_tmp[0]} meter")
        # print(f"y based camera cordinate: {tvec_tmp[1]} meter")
        # print(f"z based camera cordinate: {tvec_tmp[2]} meter")
        # print(f"yaw_angle_by_cam : {yaw_angle_by_cam} rads")
    #     # 카메라 좌표계 -> 로봇 베이스 좌표계로 변환
        point_grip = transform_cam_to_robot(tvec_tmp)
    #     print(f"before : {point_grip}")
    #     point_grip[0] = round(point_grip[0] + edit_x, 4)
    #     point_grip[1] = round(point_grip[1] + edit_y, 4)
    #     point_grip[2] = round(point_grip[2] + edit_z, 4)
        yaw_angle_by_robot = round((math.degrees(-yaw_angle_by_cam) + edit_yaw), 4)
        # yaw 변환 각도 값 추가
        point_grip = np.append(point_grip, yaw_angle_by_robot)
        point_grip = np.append(point_grip, 0)
        point_grip = np.append(point_grip, 0)
        # print(f"x pose by robot : {point_grip[0]} mm")
        # print(f"y pose by robot : {point_grip[1]} mm")
        # print(f"z pose by robot : {point_grip[2]} mm")
        # print(f"yaw angle by robot : {point_grip[3]} degree")
        print(f"after : {point_grip}")
        if ids_list[i] == [1] :
            point_grip_dic['red'] = point_grip
        elif ids_list[i] == [2] :
            point_grip_dic['green'] = point_grip
        elif ids_list[i] == [3] :
            point_grip_dic['yellow'] = point_grip
    # print(point_grip_list)
    return point_grip_dic

def ham_ArucoFixedPoint(point_grip, rate = 1.5):
    Ximg_origin, Yimg_origin = 333, 23
    X_origin, Y_origin = 286, 1.5
    point_grip[0] = X_origin + (point_grip[0] - Ximg_origin)*rate
    point_grip[1] = Y_origin + (point_grip[1] - Yimg_origin)*rate
    return point_grip

def move_HOME_position(robot_api , pose):
    
    if float(pose[1]) < 0.0:
        robot_api.client_move.MovJ(189.694461,-182, 10, 4.14)
        print("y-")
    elif float(pose[1]) >= 0.0:
        robot_api.client_move.MovJ(204.896064, 217, 10, 4.14)
        print("y+")

def move_HOME_position2(robot_api):
    robot_api.client_move.MovJ(243.87, 6.83, -6.97, 4.14)

def predict_result(model, image):
    results = model(image)
    # result_image = results[0].plot()
    detect_ob_num = results[0].boxes.cls.tolist()
    coordinate_result = results[0].boxes.xyxy.tolist()
    return results , detect_ob_num, coordinate_result

def Detect_point_aruco(ham_aruco_result1,ham_aruco_result2,ham_aruco_result3, detect_ob_class, Detect_point):
    if detect_ob_class == 'G':
        tmp2 = ham_aruco_result1
    elif detect_ob_class == 'R':
        tmp2 = ham_aruco_result2
    elif detect_ob_class == 'Y':
        tmp2 = ham_aruco_result3
    
    Detect_point[0] = tmp2[0]
    Detect_point[1] = tmp2[1]

def pick(robot_api,trans_yolo_cordi,r):
    robot_api.client_move.MovJ(trans_yolo_cordi[0], trans_yolo_cordi[1], 0, r)
    robot_api.client_move.MovL(trans_yolo_cordi[0], trans_yolo_cordi[1], trans_yolo_cordi[2], r)
    robot_api.client_dash.ToolDO(1, 1)

    robot_api.client_move.MovL(trans_yolo_cordi[0], trans_yolo_cordi[1], 0, r)

def place(robot_api,Detect_point, r):
    robot_api.client_move.MovL(Detect_point[0], Detect_point[1], -26, r)
    robot_api.client_move.MovJ(Detect_point[0], Detect_point[1], -61.5, r)
    robot_api.client_dash.ToolDO(1, 0)

def pick_stack(robot_api,Detect_point,r):
    robot_api.client_move.MovJ(Detect_point[0], Detect_point[1], -45, r)
    robot_api.client_move.MovL(Detect_point[0], Detect_point[1], -61.5, r)
    robot_api.client_dash.ToolDO(1, 1)

def place_stack(robot_api,Detect_point, r):
    robot_api.client_move.MovL(Detect_point[0], Detect_point[1], 0, r)
    robot_api.client_move.MovJ(Detect_point[0], Detect_point[1], Detect_point[2], r)
    robot_api.client_dash.ToolDO(1, 0)

    robot_api.client_move.MovJ(Detect_point[0], Detect_point[1], 0, r)

if __name__ == "__main__":
    enable_robot(ham_robotui)
    time.sleep(0.3)
    ham_robotui.client_dash.SpeedFactor(100)
    first_setting(ham_robotui)

    # set
    # start_point
    X_START = 167.54
    Y_START = -148.11

    # step
    # x_scale_robot = 0.4670833333333333
    # y_scale_robot = 0.4755625

    x_scale_robot = 0.45
    y_scale_robot = 0.45

    # offset
    X_OFFSET = -12
    Y_OFFSET = 3

    #
    # r = 4.14
    # h = -28.608
    # one_block_h = -16.5

    r = 4.14
    h = -28.45
    one_block_h = -16.55

    #
    detect_index = 0

    # default - Home
    Detect_point = [243.87, 6.83, 0, r]
    ##################
    time.sleep(0.3)
    # model load
    model = YOLO("../model/best_detect.pt")
    cap = cv2.VideoCapture(0)
    ret, img = cap.read()

    # yolo {0: 'G', 1: 'R', 2: 'Y'} // 아크로 g y r
    tmp = ham_aruco1(img=img)
    
    tmp_g = ham_ArucoFixedPoint(tmp["green"])
    tmp_r = ham_ArucoFixedPoint(tmp["red"])
    tmp_y = ham_ArucoFixedPoint(tmp["yellow"])

    tmp_y[0] = tmp_y[0] - 33
    tmp_y[1] = tmp_y[1] + 5

    tmp_r[0] = tmp_r[0] - 28
    tmp_r[1] = tmp_r[1] - 15

    tmp_g[0] = tmp_g[0] - 20
    # tmp_g[1] = tmp_g[1]

    cap.release()

    for trial in range(3) :
        cap = cv2.VideoCapture(0)
        ret, img = cap.read()

        detect_results, detect_ob_num, coordinate_result = predict_result(model, img)
        
        for idx, i in enumerate(detect_ob_num) :
            # print(predicted_results[0].names[int(idx)])
            if detect_results[0].names[int(i)] not in color_list :
                color_list.append(detect_results[0].names[int(i)])
                detect_index=idx
                break
        # print("-----------------------")
        # print(color_list)
        # print("-----------------------")    

        
        Detect_point_aruco(tmp_g,tmp_r,tmp_y, color_list[-1], Detect_point)
        
        if coordinate_result:
            trans_yolo_cordi = [X_START + ((coordinate_result[detect_index][1]+coordinate_result[detect_index][3])/2) * x_scale_robot + X_OFFSET,
                                Y_START + ((coordinate_result[detect_index][0]+coordinate_result[detect_index][2])/2) * y_scale_robot + Y_OFFSET, h + one_block_h * trial]
            
            pick(ham_robotui,trans_yolo_cordi,r)
            # print("---------1---------")
            # print(tmp_g)
            # print(tmp_r)
            # print(tmp_y)
            # print("---------2---------")
            # print(trans_yolo_cordi)
            # print("---------3---------")
            # print(Detect_point)
            place(ham_robotui,Detect_point,r)
            move_HOME_position(ham_robotui, Detect_point)

            cap.release()
            time.sleep(3.35)

    last_place_name = color_list.pop()

    for i in range(2) :
        recent_place_name = color_list[-1]

        if recent_place_name == "G":
            pick_stack(ham_robotui, tmp_g, r)
        elif recent_place_name == "R":
            pick_stack(ham_robotui, tmp_r, r)
        elif recent_place_name == "Y":
            pick_stack(ham_robotui, tmp_y, r)

        if last_place_name == "G":
            tmp_g[2] = (h + one_block_h * (1-i))
            place_stack(ham_robotui,tmp_g, r)
        elif last_place_name == "R":
            tmp_r[2] = h + one_block_h * (1-i)
            place_stack(ham_robotui,tmp_r, r)
        elif last_place_name == "Y":
            tmp_y[2] = h + one_block_h * (1-i)
            place_stack(ham_robotui,tmp_y, r)
            
        color_list.pop()

    # move_HOME_position2(ham_robotui)

    # time.sleep(2)
    # disable_robot(ham_robotui)

    # final
    # 튜닝