import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re

import serial
import time

def RunPoint(move: DobotApiMove, point_list: list):
    move.MovL(point_list[0], point_list[1], point_list[2],
              point_list[3], point_list[4], point_list[5])

class pick_place_movement: 
    def __init__(self, _pick_zero_position: np.array, _place_zero_position: np.array): 
        self.pick_zero_position = _pick_zero_position
        self.place_zero_position = _place_zero_position
        
    def move(self, _goal_position): 
        # move to pick workspace zero position
        # move to the goal position
        # move back to zero
        # move to place workspace zero position
        # move to place position
        # move to place workspace zero position
        # move to pick workspace zero position