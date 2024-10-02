#!/usr/bin/env python
# -*- coding:utf-8 -*-

#######################################################################
# Copyright (C) 2022 EunGi Han(winterbloooom) (winterbloooom@gmail.com)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>

"""Script for autonomous(obstacle avoidance) misson"""

import math
import os
import sys
import signal
import rospy
import numpy as np 
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import autonomous_visualize as av
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, UInt16
from visualization_msgs.msg import MarkerArray

import control.control_tools as control
import datatypes.point_class
import utils.gnss_converter as gc
import utils.obstacle_avoidance as oa
from ku2023.msg import ObstacleList
from utils.tools import *
import copy
from utils.PID import PID
from dock.docking_fix_2024 import *
from dock.color import ShapeColorAnalyzer
from collections import Counter
import time
class Autonomous:
    def __init__(self):
        # coordinates
        # make waypoint
        self.waypoint_idx = 1

        self.remained_waypoints = {}  # 남은 waypoints. key는 waypoint 순서, value는 [x, y] 좌표
        self.gnss_waypoint = rospy.get_param("waypoints")  # GPS 형식의 전체 waypoints
        for idx, waypoint in enumerate(self.gnss_waypoint):
            n, e, _ = gc.enu_convert(waypoint)  # ENU 좌표계로 변환
            self.remained_waypoints[idx + 1] = [n, e]  # 축이 반대이므로 순서 바꿔 할당.
        self.waypoints = copy.deepcopy(self.remained_waypoints)
        self.boat_y, self.boat_x = 0, 0  # 배의 좌표
        self.goal_x = self.remained_waypoints[self.waypoint_idx][0]  # 다음 목표 x좌표
        self.goal_y = self.remained_waypoints[self.waypoint_idx][1]  # 다음 목표 y좌표
        self.trajectory = []  # 지금껏 이동한 궤적
        self.diff = [0, 0]
    

        

        # locations, coordinates
        # self.boat_x, self.boat_y = 0, 0  # 현재 보트 위치
        # self.goal_x, self.goal_y, _ = gc.enu_convert(rospy.get_param("autonomous_goal"))  # 목표점 위치
        # self.trajectory = []  # 지금까지 이동한 궤적
        boundary = rospy.get_param("boundary1")  # 경기장 꼭짓점
        self.boundary = []
        for p in boundary:
            self.boundary.append(list(gc.enu_convert(p)))
        # self.diff = [-1.7, -0.4]  # 현재 보트 위치 GPS 보정

        # directions
        self.heading_queue = []  # 헤딩을 필터링할 이동평균필터 큐
        self.psi = 0  # 자북과 heading의 각도(자북 우측 +, 좌측 -) [degree]
        self.psi_goal = 0  # 의미 변화함: 나로부터 goal이 떨어진 각도. (+)면 오른쪽, (-)면 왼쪽에 있음
        self.psi_desire = 0  # 이번에 가야 할 각도

        # obstacles
        self.ob_dist_range = rospy.get_param("ob_dist_range")  # 라이다로 장애물을 탐지할 최대거리
        self.ob_angle_range = rospy.get_param("ob_angle_range")  # 장애물 탐지 각도
        self.span_angle = rospy.get_param("span_angle")  # 장애물 양쪽에 더해줄 각도 여유분. 충돌 방지용
        self.input_points = []  # lidar raw data
        self.obstacles = []  # 장애물 전체
        self.inrange_obstacles = []  # 탐지 범위 내 장애물
        self.danger_angles = []  # 장애물이 존재하는 각도 리스트

        # distances
        self.goal_range = rospy.get_param("goal_range")  # 도착이라 판단할 거리(반지름)
        self.distance_to_goal = 100000  # 배~목적지 거리. max 연산이므로 큰 값을 초기 할당

        # control
        self.rotate_angle_range = rospy.get_param("rotate_angle_range")  # 회전할 각도 범위
        self.servo_range = rospy.get_param("servo_range")  # 서보모터 최대/최소값
        self.servo_middle = (self.servo_range[0] + self.servo_range[1]) / 2  # 서보모터 중앙값(전진)
        self.filter_queue = []  # 서보모터값을 필터링할 이동평균필터 큐
        # self.thruster_speed = rospy.get_param("thruster_speed")  # 쓰러스터 PWM 고정값 (선속)

        # other fixed values
        self.filter_queue_size = rospy.get_param("filter_queue_size")  # 이동평균필터 큐사이즈
        self.angle_alpha = rospy.get_param("angle_alpha")  # angle_to_servo 함수에서 사용할 상수

        # visualize
        self.show_raw_pcd = rospy.get_param("show_raw_pcd")  # 라이다 raw 데이터 보이기
        self.print_cnt = 0  # 출력 속도를 조정할 카운터.

        # subscribers
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_pos_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)
        self.cam_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.cam_callback)
        self.bridge = CvBridge()
        if self.show_raw_pcd:
            self.scan_sub = rospy.Subscriber("/pointcloud/scan_data", LaserScan, self.scan_callback, queue_size=1)

        # publishers
        self.thrusterL_pub = rospy.Publisher("/thrusterL", UInt16, queue_size=0)
        self.thrusterR_pub = rospy.Publisher("/thrusterR", UInt16, queue_size=0)
        self.visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)
        self.imu_fix_pub = rospy.Publisher("/imu_fix", Float64, queue_size=1)
        self.send_psi = rospy.Publisher("/psi", Float64, queue_size=1)


        self.imu_fix = 0
        self.scale = rospy.get_param("ob_scale")

        # target info
        self.target_shape = rospy.get_param("target_shape")
        self.target_color = rospy.get_param("target_color")
        target_color_range = rospy.get_param("color_range")[self.target_color]
        self.board_color_range = rospy.get_param("board_color_range")

        self.angle_range = rospy.get_param("angle_range")  # 배열! [min, max]
        self.servo_range = rospy.get_param("servo_range")  # 배열! [min, max]
        self.servo_middle = (self.servo_range[0] + self.servo_range[1]) / 2
        self.arrival_range = rospy.get_param("arrival_range")  # 도착여부 판단할 범위
        self.color_range = np.array(
            [
                [
                    target_color_range["color1_lower"],
                    target_color_range["color2_lower"],
                    target_color_range["color3_lower"],
                ],
                [
                    target_color_range["color1_upper"],
                    target_color_range["color2_upper"],
                    target_color_range["color3_upper"],
                ],
            ]
        )
        self.board_color_array = []
        for i in self.board_color_range:
            self.board_color_array.append(np.array(
                [
                    [
                        self.board_color_range[i]["color1_lower"],
                        self.board_color_range[i]["color2_lower"],
                        self.board_color_range[i]["color3_lower"],
                    ],
                    [
                        self.board_color_range[i]["color1_upper"],
                        self.board_color_range[i]["color2_upper"],
                        self.board_color_range[i]["color3_upper"],
                    ],
                ]
            ))
        self.ref_dir_range = rospy.get_param("ref_dir_range")  # 좌우로 얼마나 각도 허용할 건가
        self.arrival_target_area = rospy.get_param("arrival_target_area")  # 도착이라 판단할 타겟 도형의 넓이
        self.mark_detect_area = rospy.get_param("mark_detect_area")  # 도형이 검출될 최소 넓이
        self.target_detect_area = rospy.get_param("target_detect_area")  # 타겟이라고 인정할 최소 넓이
        self.station_dir = rospy.get_param("station_dir")

        # coordinates
        self.enterence_x, self.enterence_y, _ = gc.enu_convert(rospy.get_param("docking_enterence"))
        self.station1_x, self.station1_y, _ = gc.enu_convert(rospy.get_param("station1"))
        self.station2_x, self.station2_y, _ = gc.enu_convert(rospy.get_param("station2"))
        self.station3_x, self.station3_y, _ = gc.enu_convert(rospy.get_param("station3"))
        self.docking_end_x, self.docking_end_y, _ = gc.enu_convert(rospy.get_param("docking_end"))

        self.boat_x, self.boat_y = 0, 0
        self.waypoints_dock = [
            [self.enterence_x, self.enterence_y],
            [self.station1_x, self.station1_y],
            [self.station2_x, self.station2_y],
            [self.station3_x, self.station3_y],
            [self.docking_end_x, self.docking_end_y]
        ]
        self.station_vec_ends = control.calc_station_vec_end(station_dir=self.station_dir, stations=self.waypoints_dock[1:])
        self.trajectory = []
        self.search_all_maybe_image_board = []
        self.search_all_maybe_image_board_list = []
        self.image_trajectory = []
        
        # data
        self.raw_img = np.zeros((480, 640, 3), dtype=np.uint8)  # row, col, channel
        self.hsv_img = np.zeros((480, 640), dtype=np.uint8)
        self.hsv_img_list =[]

        self.shape_img = np.zeros((480, 640, 3), dtype=np.uint8)
        self.mark_area = 0
        self.distance_to_point = 0  # 특정 지점으로부터 배까지의 거리

        # count
        ## (state 4에서) '이 시간동안(횟수)' 정지(약한 후진)하고 그 뒤에 헤딩 돌릴 것.
        self.stop_time = rospy.get_param("stop_time")
        ## (state 4에서) 몇 번 정지 신호를 보냈는가?
        self.stop_cnt = 2
        ## (state 5에서) '이만큼(횟수)' 기다리며 얼마나 발견하나 횟수를 셈
        self.target_detect_time = rospy.get_param("target_detect_time")
        ## (state 5에서) 그만큼 중 얼마나 기다렸는가
        self.mark_check_cnt = 0
        ## (state 5에서) 그만큼 기다리는 동안 '얼마나' 발견해야 발견이라 하겠는가
        self.target_detect_cnt = rospy.get_param("target_detect_cnt")
        ## (state 5에서) 그만큼 기다리는 동안 '몇 번' 타겟을 발견했는가
        self.detected_cnt = 0

        # constants
        self.angle_alpha = rospy.get_param("angle_alpha")
        self.pixel_alpha = rospy.get_param("pixel_alpha")

        # ON/OFF
        self.draw_contour = rospy.get_param("draw_contour")

        # speed
        # self.thruster_auto = rospy.get_param("thruster_auto")
        # self.thruster_station = rospy.get_param("thruster_station")
        # self.thruster_left = rospy.get_param("thruster_left")
        # self.thruster_right = rospy.get_param("thruster_right")
        # self.thruster_stop = rospy.get_param("thruster_stop")
        # self.thruster_back = rospy.get_param("thruster_back")

        # other settings
        self.filter_queue_size = rospy.get_param("filter_queue_size")
        self.span_angle = rospy.get_param("span_angle")
        self.ob_angle_range = rospy.get_param("ob_angle_range")
        self.ob_dist_range = rospy.get_param("ob_dist_range")
        self.find_time = rospy.get_param("find_time") #(edit) 파라미터에 추가하기

        self.thruster_speed_L= 1500
        self.thruster_speed_R= 1500
        # current status
        self.state = rospy.get_param("start_docking_state")
        # 0: 장애물 회피
        # 1: 스테이션1로 이동 중
        # 2: 스테이션2로 이동 중
        # 3: 스테이션3로 이동 중
        # 4: 헤딩 맞추는 중
        # 5: 타겟 스캔 중
        # 6: 스테이션 진입 중
        # 7: 끝. 정지
        self.target = []  # [area, center_col(pixel)]. 딕셔너리로 선언하는 쪽이 더 쉬울 듯
        self.target_found = False
        self.next_to_visit = 4  # 다음에 방문해야 할 스테이션 번호. state 시작을 1로할거면 1로
        self.imu_fix = 0


        self.square = 0
        self.triangle = 0
        self.circle = 0
        self.cross = 0
        self.max_board_count = 0
        self.use_the_board = False
        self.check_head = False
        self.check_the_three_state = False
        self.print_target_color = 0
        self.print_target_std = 0
        self.start_time = 0
        # pre-setting
        self.arrival_check()  # 다음 목표까지 남은 거리

    def cam_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if img.size == (640 * 480 * 3):
            self.raw_img = img
            # self.color_check = cv2.GaussianBlur(np.uint8([[self.raw_img[240, 320]]]), (5, 5), 0)
            # self.color_check = cv2.cvtColor(self.color_check, cv2.COLOR_BGR2HSV)


            # check_img = img.copy()
            # check_img = cv2.circle(check_img, (320, 240), 3, (255,0,0), 2)
            # cv2.imshow("check_color", check_img)
            # [120  98 189]
            #191, 115, 116
            #188 115 115
            pass   
    def check_state(self):
        change_state = False #현재 보트의 도킹 순서
        if self.state == 0: #도킹 포인트 지점으로 이동
            # 변경지점 도착 여부 판단1111
            change_state = self.calc_distance(self.waypoints_dock[0])
        elif self.state == 3:
            if self.check_the_three_state:
                change_state = True
        elif self.state == 4: # 도형 쪽으로 헤딩값 수정
            # change_state = self.check_heading()
            if self.check_head == True:
                change_state = True
        elif self.state == 5:
            if self.mark_check_cnt >= self.target_detect_time:
                change_state = True
                self.mark_check_cnt = 0
                self.detected_cnt = 0
                self.start_time2 = time.time()
            else:
                change_state = False
                
        elif self.state == 6:
            # 도킹 완료했는지 확인
            change_state = self.check_docked()

        if change_state:
            print("")
            print("{:=^70}".format(" Change State "))
            print("")
            if self.state == 0:  # 도킹 위치로 도달했을 때
                self.next_to_visit = 4 # 바로 5로 변경
                self.state = 4
                self.stop_cnt = 0
            elif self.state == 5:
                if self.target_found:
                    self.state += 1
                else:
                    self.state = 5
                    pass
            else:
                self.state += 1

            return True
        else:
            return False

    def check_heading(self):
        """선수각이 스테이션 방향을 향하고 있는지(허용 범위 내로 들어왔는지) 체크"""
        angle_to_station = self.station_dir - self.psi
        angle_to_station = rearrange_angle(angle_to_station)

        return abs(angle_to_station) <= self.ref_dir_range

    def check_target(self, return_target=False):
        """표지를 인식하고 타겟이면 타겟 정보를 반환

        Args:
            return_target (bool): target 정보를 리턴할 것인지(True), Bool 정보를 리턴할 것인지(False)

        Returns:
            True/False (bool): 타겟을 찾음(True), 찾지 못함(False)
            target (list): 타겟을 찾았고, [넓이, 중앙지점] 정보를 담고 있음
        """
        # self.show_window()
        preprocessed = mark_detect.preprocess_image(self.raw_img, blur=True , brightness=False, hsv=True)

        
        if self.target_color == 'black2':
          self.hsv_img = mark_detect.select_color(~preprocessed, self.color_range)  # 원하는 색만 필터링
        else:
            self.hsv_img = mark_detect.select_color(preprocessed, self.color_range)  # 원하는 색만 필터링


        
        #self.hsv_img = mark_detect.select_color(preprocessed, self.color_range)  # 원하는 색만 필터링

        target, self.shape_img, self.mark_area = mark_detect.detect_target(
            self.hsv_img,
            self.target_shape,
            self.mark_detect_area,
            self.target_detect_area,
            self.draw_contour,
        )  # target = [area, center_col] 형태로 타겟의 정보를 받음

        if return_target == True:
            return target
        else:
            return False if len(target) == 0 else True
        
    def check_board(self, return_target=False):
        """표지를 인식하고 타겟이면 타겟 정보를 반환

        Args:
            return_target (bool): target 정보를 리턴할 것인지(True), Bool 정보를 리턴할 것인지(False)

        Returns:
            True/False (bool): 타겟을 찾음(True), 찾지 못함(False)
            target (list): 타겟을 찾았고, [넓이, 중앙지점] 정보를 담고 있음
        """

        preprocessed = mark_detect.preprocess_image(self.raw_img, blur=True , brightness=False, hsv=True)

        self.hsv_img = preprocessed
        # try:
        #     # version1 we use to gray scale and then contour image what is shape and color
        #     target, self.shape_img, self.mark_area, img_tj, search_all= mark_detect.detect_target_state_zero_version1(
        #         self.image_trajectory,
        #         self.search_all_maybe_image_board,
        #         self.hsv_img,
        #         self.target_shape,
        #         self.mark_detect_area,
        #         self.target_detect_area,
        #         self.draw_contour,
        #     )  # target = [area, center_col] 형태로 타겟의 정보를 받음

        #     self.image_trajectory = img_tj
        #     self.search_all_maybe_image_board = search_all
        #     temp_board_count = 0
        #     for i in range(self.max_board_count+1, len(self.image_trajectory)):
        #         temp_board_count = i
        #         if self.image_trajectory[i][1] == 4:
        #             self.square += 1
        #         elif self.image_trajectory[i][1] == 3:
        #             self.triangle += 1
        #         elif self.image_trajectory[i][1] == 8:
        #             self.circle += 1
        #         elif self.image_trajectory[i][1] == 12:
        #             self.cross += 1

        #     print(self.max_board_count)
        #     self.hsv_img_list = []
        #     if len(self.image_trajectory) >= 100:
        #         self.image_trajectory = []
        #         self.max_board_count = 0
        # except:
        #     target = []
        #     pass

        #version2 we use to color_range so wo know that shape color before started autonomous
        for i in self.board_color_array:
            self.hsv_img_list.append(mark_detect.select_color(preprocessed, i))  # 원하는 색만 필터링
        for c, j in enumerate(self.hsv_img_list):
            target, self.shape_img, self.mark_area, search_all = mark_detect.detect_target_state_zero_version2(
                self.search_all_maybe_image_board,
                self.board_color_range.keys()[c],
                j,
                preprocessed,
                self.mark_detect_area,
            )  # target = [area, center_col] 형태로 타겟의 정보를 받음

            self.search_all_maybe_image_board = search_all

        temp_board_count = 0
        for i in range(self.max_board_count+1, len(self.search_all_maybe_image_board)):
            temp_board_count = i
            if self.search_all_maybe_image_board[i][0] == 4:
                self.square += 1
            elif self.search_all_maybe_image_board[i][0] == 3:
                self.triangle += 1
            elif self.search_all_maybe_image_board[i][0] == 8:
                self.circle += 1
            elif self.search_all_maybe_image_board[i][0] == 12:
                self.cross += 1

        self.max_board_count = temp_board_count
        self.hsv_img_list = []

        if return_target == True:
            return target
        else:
            return False if len(target) == 0 else True
        
    def insert_the_borad_target(self):
        target_list = [self.square, self.triangle, self.circle, self.cross]
        target_list.sort(reverse=True)
        if target_list[0] == self.square:
            self.target_shape = 4
        if target_list[0] == self.triangle:
            self.target_shape = 3
        if target_list[0] == self.circle:
            self.target_shape = 8
        if target_list[0] == self.cross:
            self.target_shape = 12

        result = self.circle + self.triangle + self.cross + self.square
        color_list = []
        if result >= 1000:
            for i in self.search_all_maybe_image_board:
                if i[0] == self.target_shape:
                    color_list.append(str(i[1]))

            color_counter = Counter(color_list)
            most_common_color = color_counter.most_common(1)
            target_color_range = rospy.get_param("color_range")[most_common_color[0][0]]
            color_range = np.array(
            [
                [
                    target_color_range["color1_lower"],
                    target_color_range["color2_lower"],
                    target_color_range["color3_lower"],
                ],
                [
                    target_color_range["color1_upper"],
                    target_color_range["color2_upper"],
                    target_color_range["color3_upper"],
                ],
            ]
            )
            self.color_range = color_range
            self.check_the_three_state = True
    def calc_distance2(self, point):
        self.distance_to_point = math.hypot(self.boat_x - point[0], self.boat_y - point[1])

        return self.distance_to_point <= self.arrival_range

    def check_docked(self):
        """스테이션에 도크되었는지 확인

        도킹 모드(6번)에서 마크를 탐지했을 때, 탐지가 되었다면 마크의 넓이를 기준으로 판단.
        탐지가 되지 않았다면 도크되지 않았다고 판단.

        Returns:
            True/False: 도킹 끝(True), 아직 안 끝남(False)
        """
        if self.calc_distance2(self.waypoints_dock[4]):# (edit) clac_distance2 수정 필요 coordinates.yaml 수정필요 param 수정 필요
            return True
        else:
            return False



        if len(self.target) != 0:
            return self.target[0] >= self.arrival_target_area
        else:
            return False

    def calc_psi_goal_one(self):
        """다음 목표지점으로 가기 위한 선수 회전 각도를 계산"""
        psi_goal = (
            math.degrees(
                math.atan2(
                    self.waypoints[0][1] - self.boat_y,
                    self.waypoints[0][0] - self.boat_x,
                )
            )
            - self.psi
        )
        self.psi_goal = rearrange_angle(psi_goal)

    def calc_psi_goal_four(self):
        """다음 목표지점으로 가기 위한 선수 회전 각도를 계산"""
        psi_goal = (
            math.degrees(
                math.atan2(
                    self.waypoints_dock[4][1] - self.boat_y,
                    self.waypoints_dock[4][0] - self.boat_x,
                )
            )
            - self.psi
        )
        self.psi_goal = rearrange_angle(psi_goal)

    def print_status2(self, error_angle, u_servo, u_thruster):
        state_str = [
            "Avoiding Obstacles",
            "Going to Station #1",
            "Going to Station #2",
            "Going to Station #3",
            "Rotating Heading",
            "Detecting Target",
            "Docking",
            "End",
        ]
        print("")
        print("({:>4.2f}, {:>4.2f})".format(self.boat_x, self.boat_y))
        print("State: # {} - {}  angle = {}".format(str(self.state), state_str[self.state] , error_angle))
        print("")

        if self.state == 6:
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"
            print("Mark Area    : {:>7,.0f} / {:>7,.0f}".format(self.mark_area, self.mark_detect_area))
            print(
                "Target Area  : {:>7,.0f} / {:>7,.0f} ({:>5})".format(
                    self.target[0] if len(self.target) != 0 else 0,
                    self.target_detect_area,
                    "Found" if len(self.target) != 0 else "None",
                )
            )
            print(
                "Arrival Area : {:>7,.0f} / {:>7,.0f}".format(
                    self.target[0] if len(self.target) != 0 else 0,
                    self.arrival_target_area,
                )
            )
            print("")
            print("mid - {:>6} = {:>11} {:->4} {:>11}".format("target", "error_pixel", ">", "error_angle"))
            print(
                "320 - {:>6,.0f} = {:>11,.0f} {:>4} {:>11.2f} {:>9}".format(
                    self.target[1] if len(self.target) != 0 else 0,
                    320 - self.target[1] if len(self.target) != 0 else 0,
                    "",
                    error_angle,
                    error_angle_dir_str,
                )
            )
            print("")

        if self.state in [0, 1, 2, 3]:
            psi_goal_dir_str = "[   | * ]" if self.psi_goal > 0 else "[ * |   ]"
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"
            if u_servo > self.servo_middle:
                servo_value_str = str("<" * int(((self.servo_middle - u_servo) // 5)))  # go left
            else:
                servo_value_str = str(">" * int(((self.servo_middle - u_servo) // 5)))   # go right
            print("{:^9}   {:^8} - {:^8} = {:^8} {:->9} {:^5}".format("goal", "desire", "psi", "error", ">", "servo"))
            print(
                "{:>9}   {:>8.2f} - {:>8.2f} = {:>8.2f} {:>9} {:>5} ( {:^5} )".format(
                    psi_goal_dir_str,
                    self.psi_desire,
                    self.psi,
                    error_angle,
                    error_angle_dir_str,
                    u_servo,
                    servo_value_str,
                )
            )
            print("")
            print("{:<9} : {:6.2f} m".format("distance", self.distance_to_point))

        elif self.state == 4:
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"

            if u_servo > self.servo_middle:
                servo_value_str = "<" * ((self.servo_middle - u_servo) // 5)  # go left
            else:
                servo_value_str = ">" * ((self.servo_middle - u_servo) // 5)  # go right

            if self.stop_cnt >= self.stop_time:
                print("Rotating Heading >>>>")
            else:
                print("Stopping Boat >>>>>>> {:>2d} / {:>2d}".format(self.stop_cnt, self.stop_time))
            print("")
            print("{:^8} - {:^8} = {:^8} {:->9} {:^5}".format("desire", "psi", "error", ">", "servo"))
            print(
                "{:>8.2f} - {:>8.2f} = {:>8.2f} {:>9} {:>5} ( {:^5} )".format(
                    self.psi_desire,
                    self.psi,
                    error_angle,
                    error_angle_dir_str,
                    u_servo,
                    servo_value_str,
                )
            )

        elif self.state == 5:
            print("Target Shape : {} | Color : {}".format(self.target_shape, self.target_color))
            print("Waiting..... : {:>4d} / {:>4d}".format(self.mark_check_cnt, self.target_detect_time))
            print("Target Cnt   : {:>4d} / {:>4d}".format(self.detected_cnt, self.target_detect_cnt))
            print("")
            print("Mark Area    : {:>7,.0f} / {:>7,.0f}".format(self.mark_area, self.mark_detect_area))

        print("")
        print("ThrusterL  : {}".format(self.thruster_speed_L))
        print("ThrusterR  : {}".format(self.thruster_speed_R))

        print("")
        try:
            print("square: {}, triangle: {}, circle {},  cross {}".format(self.square, self.triangle, self.circle,  self.cross))
            print("target: {}".format(self.target_shape))
            print("target_color: {}, std_color {}".format(self.print_target_color , self.print_target_std))
            print("color range {} , {}".format(self.color_range[0], self.color_range[1]))
        except:
            pass
        print("\n\n\n\n")

        print("-" * 70)

    def get_trackbar_pos(self):
        """get trackbar poses and set each values"""
        # self.color_range[0][0] = cv2.getTrackbarPos("color1 min", "controller")
        # self.color_range[1][0] = cv2.getTrackbarPos("color1 max", "controller")
        # self.color_range[0][1] = cv2.getTrackbarPos("color2 min", "controller")
        # self.color_range[1][1] = cv2.getTrackbarPos("color2 max", "controller")
        # self.color_range[0][2] = cv2.getTrackbarPos("color3 min", "controller")
        # self.color_range[1][2] = cv2.getTrackbarPos("color3 max", "controller")
        # self.mark_detect_area = cv2.getTrackbarPos("mark_detect_area", "controller")
        # self.target_detect_area = cv2.getTrackbarPos("target_detect_area", "controller")
        # self.arrival_target_area = cv2.getTrackbarPos("arrival_target_area", "controller")

    def show_window(self):
        self.get_trackbar_pos()
        cv2.moveWindow("controller", 0, 0)
        if self.state in [5, 6]:
            raw_img = cv2.resize(self.raw_img, dsize=(0, 0), fx=0.5, fy=0.5)  # 카메라 데이터 원본
            hsv_img = cv2.resize(self.hsv_img, dsize=(0, 0), fx=0.5, fy=0.5)  # 색 추출 결과
            hsv_img = cv2.cvtColor(hsv_img, cv2.COLOR_GRAY2BGR)
            col1 = np.vstack([raw_img, hsv_img])
            col2 = cv2.resize(self.shape_img, dsize=(0, 0), fx=0.9, fy=1.0)  # 타겟 검출 결과
            show_img = np.hstack([col1, col2])
            cv2.imshow("controller", show_img)
        else:
            raw_img = cv2.resize(self.raw_img, dsize=(0, 0), fx=0.5, fy=0.5)
            cv2.imshow("controller", raw_img) 
            cv2.imshow("shape", self.shape_img) 

    def scan_callback(self, msg): ##/scan 토픽 안에 range_min, range_max값이 있음??...?
        if not self.show_raw_pcd:
            return

        self.input_points = []
        phi = msg.angle_max # 각 점의 각도 계산 위해 계속 누적해갈 각도
        for r in msg.ranges:
            if msg.range_min/2 <= r <= msg.range_max/2:
                p = datatypes.point_class.Point.polar_to_cartesian(r, phi)
                self.input_points.append(p)
            phi += msg.angle_increment

    def heading_callback(self, msg):
        """IMU 지자기 센서로 측정한 자북과 heading 사이각 콜백함수

        Args:
            msg (Float64) : heading. 0 = magnetic north, (+) = 0~180 deg to right, (-) = 0 ~ -180 deg to left

        Notes:
            * IMU의 예민성으로 인하여 heading 값에 noise가 있음. 따라서 이동평균필터를 적용함.
        """
        # self.psi = moving_avg_filter(
        #     self.heading_queue, self.filter_queue_size, msg.data
        # )  # [deg]
        self.psi = -(msg.data - self.imu_fix)
        self.send_psi.publish(self.psi)
        # self.psi = -(msg.data - 50)

    def boat_position_callback(self, msg):
        """GPS로 측정한 배의 ENU 변환 좌표 콜백함수

        Args:
            msg (Point) : position of boat

        Note:
            * ENU좌표계로 변환되어 입력을 받는데, ENU좌표계와 x, y축이 반대임
            * 따라서 Point.x, Point.y가 각각 y, x가 됨
        """
        self.boat_x = msg.x + self.diff[0]
        self.boat_y = msg.y + self.diff[1]

    def obstacle_callback(self, msg):
        """lidar_converter에서 받아온 장애물 정보 저장

        Args:
            msg (ObstacleList) : list of obstacles(Obstacle object)

        Note:
            * 개당 [msg.obstacle.begin.x, msg.obstacle.begin.y, msg.obstacle.end.x, msg.obstacle.end.y]
        """
        self.obstacles = msg.obstacle

    def is_all_connected(self):

        rospy.wait_for_message("/heading", Float64)
        print("\n{:><70}".format("heading_calculator Connected "))
        rospy.wait_for_message("/enu_position", Point)
        print("\n{:><70}".format("gnss_converter Connected "))
        # rospy.wait_for_message("/obstacles", ObstacleList)
        # print("\n{:><70}".format("lidar_converter Connected "))
        # if self.show_raw_pcd:
        #     rospy.wait_for_message("/scan", LaserScan)
        #     print("\n{:><70}".format("LiDAR Connected "))
        return True

    def arrival_check(self):
        """calculate distance from boat to the next goal of current state

        Returns:
            bool : True (= arrived to the goal) / False
        """
        self.distance_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)
        return self.distance_to_goal <= self.goal_range

    def print_status(self, error_angle, u_servo): 
        """print current state

        Args:
            error_angle (float) : angle between psi_desire and psi (heading to desire angle)
            u_servo (int) : servo moter publish value
        """
        print("")
        print("({:>4.2f}, {:>4.2f})".format(self.boat_x, self.boat_y))
        print("Obstacle  : {:2d} / {:2d}".format(len(self.inrange_obstacles), len(self.obstacles)))

        psi_goal_dir_str = "[   | * ]" if self.psi_goal > 0 else "[ * |   ]"
        error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"
        if u_servo > self.servo_middle:
            servo_value_str = "<" * ((self.servo_middle - u_servo) // 5)  # go left
        else:
            servo_value_str = ">" * ((self.servo_middle - u_servo) // 5)  # go right

        print("")
        print("{:^9}   {:^6} - {:^6} = {:^6} {:->9} {:^5}".format("goal", "desire", "psi", "error", ">", "servo"))
        print(
            "{:>9}   {:>6.2f} - {:>6.2f} = {:>6.2f} {:>9} {:>5} ( {:^5} )".format(
                psi_goal_dir_str,
                self.psi_desire,
                self.psi,
                error_angle,
                error_angle_dir_str,
                u_servo,
                servo_value_str,
            )
        )
        print("")
        print("{:<9} : {:6.2f} m".format("distance", self.distance_to_goal))
        print("")
        print("-" * 70)

    def set_next_goal(self):
        del self.remained_waypoints[self.waypoint_idx]
        self.waypoint_idx += 1

        if len(self.gnss_waypoint) + 1 == self.waypoint_idx:
            return

        self.goal_x = self.remained_waypoints[self.waypoint_idx][0]
        self.goal_y = self.remained_waypoints[self.waypoint_idx][1]

def shutdown():
    auto.thrusterL_pub.publish(1500)
    auto.thrusterR_pub.publish(1500)
    auto.thrusterL_pub.publish(1500)
    auto.thrusterR_pub.publish(1500)
    print("press ctrl c ")
    sys.exit(0)


def main():
    global auto
    rospy.init_node("autonomous", anonymous=False)
    start_time = rospy.get_time()
    auto = Autonomous()
    rate = rospy.Rate(10)
    util_n =37 ## 5~7
    fix_imu=0.0
    imu_fix = True

    while not auto.is_all_connected():
        rospy.sleep(0.2)
    print("\n{:<>70}".format(" All Connected !"))
    while not rospy.is_shutdown():
        if len(auto.remained_waypoints) == 0:# 마지막 목적지까지 도착함
            auto.thrusterL_pub.publish(1500)
            auto.thrusterR_pub.publish(1500)
            auto.thrusterL_pub.publish(1500)
            auto.thrusterR_pub.publish(1500)
            rospy.sleep(1.5)
            print("-" * 20)
            print("Finished!")
            return
        else:
            
            # auto.trajectory.append([auto.boat_x, auto.boat_y])         
            arrived = auto.arrival_check()  # 현 시점에서 목표까지 남은 거리 재계산
            if arrived:  # current goal in and change goal
                start = time.time()
                while True:
                    end = time.time()
                    stop_time = end-start
                    self_stop_time  = stop_time
                    print(stop_time)
                    if stop_time < 1:    
                        thruster_speed_L = 1400    
                        thruster_speed_R = 1400
                        auto.thrusterL_pub.publish(1430)
                        auto.thrusterR_pub.publish(1400)
                    elif stop_time < 3:
                        thruster_speed_L = 1500    
                        thruster_speed_R = 1500
                        auto.thrusterL_pub.publish(1500)
                        auto.thrusterR_pub.publish(1500)
                    else:
                        auto.set_next_goal()
                        break
                    # rate.sleep()
                    rospy.sleep(0.1)

            else:
                if auto.waypoint_idx == 3:
                    docking_part(auto)
                    auto.set_next_goal() 
                else:
                    auto.trajectory.append([auto.boat_x, auto.boat_y])  # 이동 경로 추가
                    ###move and add the next goal
                    # 현재 heading에서 목표로 갈 때 돌려야 할 각도 업데이트
                    auto.psi_goal = math.degrees(math.atan2(auto.goal_y - auto.boat_y, auto.goal_x - auto.boat_x)) - auto.psi
                    print("first", auto.psi_goal)
                    auto.psi_goal = rearrange_angle(auto.psi_goal)
                    print("second", auto.psi_goal)

                    if auto.waypoint_idx == 1 and imu_fix:
                        auto.imu_fix = auto.psi_goal
                        imu_fix= False

                    # 장애물 탐지. 범위 내에 있는 장애물을 필터링하고, 장애물이 있는 각도 리스트를 만듦
                    auto.inrange_obstacles, auto.danger_angles = oa.ob_filtering(
                        obstacles=auto.obstacles,
                        dist_to_goal=auto.distance_to_goal,
                        angle_to_goal=auto.psi_goal,
                        span_angle=auto.span_angle,
                        angle_range=auto.ob_angle_range,
                        distance_range=auto.ob_dist_range,
                        scale=auto.scale,
                    )

                    # 목표각과 현 헤딩 사이 상대적 각도 계산. 선박고정좌표계로 '가야 할 각도'에 해당
                    error_angle = oa.calc_desire_angle(
                        danger_angles=auto.danger_angles,
                        angle_to_goal=auto.psi_goal,
                        angle_range=auto.ob_angle_range,
                    )

                    # 월드좌표계로 '가야 할 각도'를 계산함
                    auto.psi_desire = rearrange_angle(auto.psi + error_angle)

                    # degree 단위를 servo moter 단위로 변경
                    u_servo = control.degree_to_servo(
                        error_angle=error_angle,
                        angle_alpha=auto.angle_alpha,
                        angle_range=auto.rotate_angle_range,
                        servo_range=auto.servo_range,
                    )
                    # u_servo = moving_avg_filter(auto.filter_queue, auto.filter_queue_size, u_servo)
                    angle_PID = PID()
                    distance_PID = PID()

                    PID_angle = angle_PID.update(error_angle)
                    PID_distance = distance_PID.update(auto.distance_to_goal)
                    #-----------------edit----------------------------------------------------#

                    thruster_speed_L=1650
                    thruster_speed_R=1650
                    limit_go_speed = 1900
                    limit_back_speed = 1100
                    PID_distance_value = 8

                    #------------------------------67-------------------------------------------#

                    PID_distance = int(abs(math.log(pow(PID_distance, PID_distance_value), 2)))
                    PID_angle = int(PID_angle)
                    if error_angle < 0:
                        thruster_speed_L = thruster_speed_L + abs(PID_angle)
                        thruster_speed_R = thruster_speed_R - abs(PID_angle)
                    else:
                        thruster_speed_L = thruster_speed_L - abs(PID_angle)
                        thruster_speed_R = thruster_speed_R + abs(PID_angle)

                    # thruster_speed_L = thruster_speed_L + PID_distance
                    # thruster_speed_R = thruster_speed_R + PID_distance


                    # if error_angle > -1.0 and error_angle < 0.0: #go light 
                    #     thruster_speed_L= 1560
                    #     thruster_speed_R= 1440
                    # elif error_angle<1.0 and error_angle>0.0: #go left
                    #     thruster_speed_L= 1440
                    #     thruster_speed_R= 1560

                    # if error_angle > -1.0 and error_angle < -2: #go light 
                    #     thruster_speed_L= 1580
                    #     thruster_speed_R= 1420
                    # elif error_angle<1.0 and error_angle>2: #go left
                    #     thruster_speed_L= 1420
                    #     thruster_speed_R= 1580

                    # elif error_angle > -2.0 and error_angle < -3.0: #go light 
                    #     thruster_speed_L= 1600
                    #     thruster_speed_R= 1410
                    # elif error_angle<2.0 and error_angle>3.0: #go left
                    #     thruster_speed_L= 1410
                    #     thruster_speed_R= 1600

                    # elif error_angle < -4.0: #go left 
                    #     thruster_speed_L= 1650
                    #     thruster_speed_R= 1400
                    # elif error_angle>4.0: #go left
                    #     thruster_speed_L= 1400
                    #     thruster_speed_R= 1650

                    #----------------------------------------------------------------------------

                    # 제어명령
                    if thruster_speed_L > limit_go_speed:
                        thruster_speed_L = limit_go_speed
                    if thruster_speed_R > limit_go_speed:
                        thruster_speed_R = limit_go_speed
                    if thruster_speed_L < limit_back_speed:
                        thruster_speed_L = limit_back_speed
                    if thruster_speed_R < limit_back_speed:
                        thruster_speed_R = limit_back_speed

                    print("PID_angle", PID_angle)
                    print("PID_distance", PID_distance)

                    print("thruster_speed_L", int(thruster_speed_L))
                    print("thruster_speed_R", int(thruster_speed_R))

                    auto.thrusterL_pub.publish(int(thruster_speed_L))
                    auto.thrusterR_pub.publish(int(thruster_speed_R))

                    

                    # 현 상태 출력 및 시각화
                    print("")
                    print("{:<9} : {:<6.3f}".format("Run time", rospy.get_time() - start_time))  # 작동 시간
                    auto.print_status(error_angle, u_servo)
                    auto.imu_fix_pub.publish(auto.imu_fix)
                    all_markers = av.visualize(auto)
                    auto.visual_rviz_pub.publish(all_markers)
            rate.sleep()
    rospy.on_shutdown(shutdown)
    



if __name__ == "__main__":
    main()
