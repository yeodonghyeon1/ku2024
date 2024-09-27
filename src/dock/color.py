#!/usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np 
import cv2 
from collections import defaultdict 
from scipy.stats import norm 

class ShapeColorAnalyzer: 
    def __init__(self): 
        self.search_all_maybe_image_board = [] 
        # [변의 개수, 픽셀 색상]이 저장된 리스트 
        self.color_by_shape = defaultdict(list) 
        # 도형별 색상 저장을 위한 딕셔너리 
    def analyze_colors(self): 
        # 변의 개수별로 색상을 그룹화 
        for shape_info in self.search_all_maybe_image_board: 
            edges = shape_info[0] 
            color = shape_info[1] 
            self.color_by_shape[edges].append(color) 
            # 각 도형에 대해 정규분포를 이용해 가장 빈번한 색상을 계산 
        color_distribution = {} 
        for edges, colors in self.color_by_shape.items(): 
            if len(colors) > 0: 
                # 색상 리스트를 numpy 배열로 변환 
                colors = np.array(colors) 
                # 각 색상 채널(R, G, B)의 평균 계산 
                mean_colors = np.mean(colors, axis=0) 
                # 각 채널의 표준편차 계산 
                std_colors = np.std(colors, axis=0) 
                # 정규분포를 사용해 가장 빈번한 색상을 저장 
                color_distribution[edges] = (mean_colors, std_colors) 
        return color_distribution 
    def insert_the_borad_target(self, target): 
            # 리스트가 5000개 이상이면 색상 분포 분석 실행 
        color_distribution = self.analyze_colors() 
        # 예: 사각형(변 4개)의 평균 색상 가져오기 
        if target in color_distribution: 
            self.mean_color, self.std_color = color_distribution[target] 
            self.check_the_three_state = True 
        # if 8 in color_distribution: 
        #     mean_color_square, std_color_square = color_distribution[8] 
        #     print("cicle의 평균 색상: {}, 표준편차: {}".format(mean_color_square,std_color_square )) 
        #     self.check_the_three_state = True 
        # if 12 in color_distribution: 
        #     mean_color_square, std_color_square = color_distribution[12] 
        #     print("cross의 평균 색상: {}, 표준편차: {}".format(mean_color_square,std_color_square )) 
        #     self.check_the_three_state = True 
        # if 3 in color_distribution: 
        #     mean_color_square, std_color_square = color_distribution[3] 
        #     print("triangle의 평균 색상: {}, 표준편차: {}".format(mean_color_square,std_color_square )) 
        #     self.check_the_three_state = True 