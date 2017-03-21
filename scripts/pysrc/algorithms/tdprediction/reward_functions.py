"""
A collection of reward functions.
"""

import numpy as np

__author__ = 'kongaloosh'


def load_2(data):
    return (data.load_2 + 300)/600


def temperature_2(data):
    return data.temperature_2/100


def voltage_2(data):
    return data.voltage_2/50


def is_moving_2(data):
    return data.is_moving_2


def poisiton_2(data):
    return data.position_2/1024


def angle_2(data):
    return (data.angle_2 + 4)/8


def vel_command_2(data):
    return data.vel_command_2 /2


def load_3(data):
    return data.load_3


def temperature_3(data):
    return data.temperature_3


def voltage_3(data):
    return data.voltage_3


def is_moving_3(data):
    return data.is_moving_3


def poisiton_3(data):
    return data.position_3


def angle_3(data):
    return data.angle_3


def vel_command_3(data):
    return data.vel_command_3

def command(data):
    return data.command

