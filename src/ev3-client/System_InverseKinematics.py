#!/usr/bin/env python3

import math
import time
from threading import Thread
from System_constT import *
from ev3dev2.motor import SpeedPercent
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D

def run_motor(motor, speed, degrees):
        motor.on_for_degrees(SpeedPercent(speed), degrees)

# TODO:

motor_a = LargeMotor(OUTPUT_B)
motor_b = LargeMotor(OUTPUT_C)
motor_c = LargeMotor(OUTPUT_D)
motor_d = LargeMotor(OUTPUT_A)

print("Initializing the System")
system = System(motor_a, motor_b, motor_c, motor_d)

while True:

        print()
        x = float(input("X Coordinate: "))
        y = float(input("Y Cooridnate: "))

        if (7 <= x <= 53 and 7 <= y <= 53):
                print("Status: Setting up Motion")
                system.update_target_using_coords((x,y))
                system.update_command_buffer()
                system.move_ee()
                print("Status: Motion Completed" + "\n")
                print()

        else:
                print("Point outside Workspace")
                print("Try Again")

        condition = input("Continue ?")
        
        if (condition == "n"):
                print("System Return")
                system.move_ee_to_centre()
                break