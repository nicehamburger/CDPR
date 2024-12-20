#!/usr/bin/env python3

import math
import time
from threading import Thread
from ev3dev2.motor import SpeedPercent, SpeedRPM
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D

class System:
    
    # System Contants (in cm)
    base_frame_length  = 61
    base_frame_width   = 61
    ee_frame_length    = 6
    ee_frame_width     = 4
    pradius            = (4*1.6) / (2*(math.pi))  # pulley radius approximation

    # System Variables
    a = [0, base_frame_length]
    b = [base_frame_width, base_frame_length]
    c = [base_frame_width, 0]
    d = [0, 0]
    frame = [a,b,c,d]           # system frame
    centre_of_system = [base_frame_width/2 , base_frame_length/2]

    # Defining End Effector Variables, Current Position
    o             = []  # End Effector Centre Position
    points        = []  # End Effector Frame Coordinates [[xP,yP],[xQ,yQ],[xR,yR],[xS,yS]]
    currentlength = []  # Cable lengths associated with current EE Position [LA,LB,LC,LD]

    # Defining Target Position
    disp = (0,0)        # Displacement with respect to current EE Centre Position
    new  = []           # Target Position of EE Centre with respect to Base Frame Coordinates
    new_points = []     # End Effector Frame Coordinates required for Target EE Centre Position
    newlength  = []     # Cable lengths associated with EE for acheiving Target EE Centre Position

    # Finding Movement Variables
    angles        = []  # Absolute required Angles in degrees
    largest_angle = 0   # Largest Angle in each System Iteration (Each EE movement called by user)
    t             = 0   # Defining Constant Time variable for motor motion assoicated with a System Iteration

    # Command Buffer Variables
    min_vel = 5         # Minimum motor velocity
    max_vel = 30        # Maximum motor velocity
    motor_a = motor_b = motor_c = motor_d = None
    command_buffer = None

    def __init__(self, motor_a, motor_b, motor_c, motor_d):
        
        # Initializing End Effector Centre and Frame Coordinates Variables (Assuming EE at Centre of System)
        self.o = [self.base_frame_width/2 , self.base_frame_length/2]
        p = [self.o[0] - self.ee_frame_width/2 , self.o[1] + self.ee_frame_length/2]
        q = [self.o[0] + self.ee_frame_width/2 , self.o[1] + self.ee_frame_length/2]
        r = [self.o[0] + self.ee_frame_width/2 , self.o[1] - self.ee_frame_length/2]
        s = [self.o[0] - self.ee_frame_width/2 , self.o[1] - self.ee_frame_length/2]
        self.points = [p,q,r,s] # End Effector Frame Coordinates [[xP,yP],[xQ,yQ],[xR,yR],[xS,yS]]

        # Initiliazing Cable lengths
        for i in range(0,4):
            diff_x = self.points[i][0] - self.frame[i][0]   # Difference in x coordinate
            diff_y = self.points[i][1] - self.frame[i][1]   # Difference in y coordinate
            self.currentlength.append(math.sqrt(diff_x**2 + diff_y**2))
        print("Initial Cable Length:",self.currentlength)
        # Initialize motors and command buffer
        self.motor_a = motor_a
        self.motor_b = motor_b
        self.motor_c = motor_c
        self.motor_d = motor_d
        self.command_buffer = [
            [motor_a, self.min_vel, 0],
            [motor_b, self.min_vel, 0],
            [motor_c, self.min_vel, 0],
            [motor_d, self.min_vel, 0]
        ]

    def update_target_using_coords(self, coords):
        
        # Target EE Centre Position defined as Coordinates given with respect to Base Frame
        self.new = coords
        new_p = [self.new[0] - self.ee_frame_width/2 , self.new[1] + self.ee_frame_length/2]
        new_q = [self.new[0] + self.ee_frame_width/2 , self.new[1] + self.ee_frame_length/2]
        new_r = [self.new[0] + self.ee_frame_width/2 , self.new[1] - self.ee_frame_length/2]
        new_s = [self.new[0] - self.ee_frame_width/2 , self.new[1] - self.ee_frame_length/2]
        self.new_points = [new_p, new_q, new_r, new_s]
        newlength = []
        for i in range(0,4):
            diff_x = self.new_points[i][0] - self.frame[i][0]
            diff_y = self.new_points[i][1] - self.frame[i][1]
            newlength.append(math.sqrt(diff_x**2 + diff_y**2))
        print("LA:",newlength[0]," LB:",newlength[1]," LC:",newlength[2]," LD:",newlength[3])
        self.newlength = newlength

    def update_target_using_disp(self, displacement):
        
        # Target EE Centre Position defined using Displacement from current EE Centre Position
        self.disp = displacement
        self.new  = [self.base_frame_width/2 + displacement[0], self.base_frame_length/2 + displacement[1]]
        new_p = [self.new[0] - self.ee_frame_width/2 , self.new[1] + self.ee_frame_length/2]
        new_q = [self.new[0] + self.ee_frame_width/2 , self.new[1] + self.ee_frame_length/2]
        new_r = [self.new[0] + self.ee_frame_width/2 , self.new[1] - self.ee_frame_length/2]
        new_s = [self.new[0] - self.ee_frame_width/2 , self.new[1] - self.ee_frame_length/2]
        self.new_points = [new_p, new_q, new_r, new_s]
        newlength = []
        for i in range(0,4):
            diff_x = self.new_points[i][0] - self.frame[i][0]
            diff_y = self.new_points[i][1] - self.frame[i][1]
            newlength.append(math.sqrt(diff_x**2 + diff_y**2))
        print("LA:",newlength[0]," LB:",newlength[1]," LC:",newlength[2]," LD:",newlength[3])
        self.newlength = newlength

    def update_req_angles(self):
        
        # Calculating (Absolute) Angles by which each motor has to spin for acheiving Target position
        abs_req_angles = []
        max_angle      = 0
        # angle = cable length difference / pulley radius
        for i in range(0,4):
            angle = abs(self.newlength[i] - self.currentlength[i]) / self.pradius
            deg_angle = math.degrees(angle)
            abs_req_angles.append(deg_angle)
            if deg_angle > max_angle:
                max_angle = deg_angle   # Finding the biggest angle among the four
        self.largest_angle = max_angle
        self.angles = abs_req_angles
        self.command_buffer = [
            [self.motor_a, self.min_vel, self.angles[0]],
            [self.motor_b, self.min_vel, self.angles[1]],
            [self.motor_c, self.min_vel, self.angles[2]],
            [self.motor_d, self.min_vel, self.angles[3]]
        ]
        self.t = self.largest_angle / self.max_vel # Finding constant time for each System Iteration
        
    def fix_command_buffer(self):
        
        # Correcting Command Buffer Angles to Push/Pull (+ve/-ve angles) with respect to Motor Orientation
        # Correcting Command Buffer Velocity to prevent Cable Slacking (Variable Velocity for each Motor)

        # Motor A
        # =======
        # Setting Angle
        command_a = self.command_buffer[0]
        if (self.newlength[0] > self.currentlength[0]):
            command_a[2] *= -1
        elif (self.newlength[0] < self.currentlength[0]):
            command_a[2] *= 1
        else:
            command_a[2]  = 0
        # Setting Speed
        if self.angles[0] == self.largest_angle:
            command_a[1]  = self.max_vel
        else:
            command_a[1]  = self.max_vel * self.angles[0] / self.largest_angle

        # Motor B
        # =======
        # Setting Angle
        command_b = self.command_buffer[1]
        if (self.newlength[1] > self.currentlength[1]):
            command_b[2] *= -1
        elif (self.newlength[1] < self.currentlength[1]):
            command_b[2] *= 1
        else:
            command_b[2]  = 0
        # Setting Speed
        if self.angles[1] == self.largest_angle:
            command_b[1]  = self.max_vel
        else:
            command_b[1]  = self.max_vel * self.angles[1] / self.largest_angle

        # Motor C
        # =======
        # Setting Angle
        command_c = self.command_buffer[2]
        if (self.newlength[2] > self.currentlength[2]):
            command_c[2] *= -1
        elif (self.newlength[2] < self.currentlength[2]):
            command_c[2] *= 1
        else:
            command_c[2]  = 0
        # Setting Speed
        if self.angles[2] == self.largest_angle:
            command_c[1]  = self.max_vel
        else:
            command_c[1]  = self.max_vel * self.angles[2] / self.largest_angle

        # Motor D
        # =======
        # Setting Angle
        command_d = self.command_buffer[3]
        if (self.newlength[3] > self.currentlength[3]):
            command_d[2] *= -1
        elif (self.newlength[3] < self.currentlength[3]):
            command_d[2] *= 1
        else:
            command_d[2]  = 0
        # Setting Speed
        if self.angles[3] == self.largest_angle:
            command_d[1]  = self.max_vel
        else:
            command_d[1]  = self.max_vel * self.angles[3] / self.largest_angle

        self.command_buffer = [command_a, command_b, command_c, command_d]
   
    def update_command_buffer(self):

        # Updating Angles
        self.update_req_angles()
        # Decide Push (Release Cable) or Pull (Withdraw Cable) for current System Iteration
        self.fix_command_buffer()

    def move_ee(self):
        
        def run_motor(motor, speed, degrees):
            motor.on_for_degrees(SpeedPercent(speed), degrees)
        
        print("Status: End Effector Movement Started")
        threads = []
        for command in self.command_buffer:
            thread = Thread(target=run_motor, args=(command[0], command[1], command[2]))
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join()
        print("Status: End Effector Movement Completed")
        
        # Update Current EE Position
        self.o = self.new
        self.points = self.new_points
        self.currentlength = self.newlength

        time.sleep(4)

    def move_ee_to_centre(self):
        
        self.update_target_using_coords(self.centre_of_system)
        self.update_command_buffer()
        self.move_ee()
    
    # Variable Velocity Method 2 [Not Used]

    def variable_velocity_movement(self,coords):
        
        self.update_target_using_coords(coords)
        ee_vel = 10 # cm/s
        d = math.sqrt( (self.o[0] - self.new[0])**2 + (self.o[1] - self.new[1]**2) )
        t = d / ee_vel  # cm / cm/s = s => time in seconds

        motor_vel = []
        abs_req_angles = []

        # angle = cable length difference / pulley radius
        # w     = angle / t

        for i in range(0,4):
            angle = abs(self.newlength[i] - self.currentlength[i]) / self.pradius # ==> in radians
            w     = angle / t # ==> in rad/s
            corrected_w = (w * 60 / (2*math.pi)) # ==> in RPM
            deg_angle = math.degrees(angle)
            abs_req_angles.append(deg_angle)
            motor_vel.append(corrected_w)
            
        self.angles = abs_req_angles
        self.command_buffer = [
            [self.motor_a, motor_vel[0], self.angles[0]],
            [self.motor_b, motor_vel[1], self.angles[1]],
            [self.motor_c, motor_vel[2], self.angles[2]],
            [self.motor_d, motor_vel[3], self.angles[3]]
        ]

        # Motor A
        # =======
        # Setting Angle
        command_a = self.command_buffer[0]
        if (self.newlength[0] > self.currentlength[0]):
            command_a[2] *= -1
        elif (self.newlength[0] < self.currentlength[0]):
            command_a[2] *= 1
        else:
            command_a[2]  = 0

        # Motor B
        # =======
        # Setting Angle
        command_b = self.command_buffer[1]
        if (self.newlength[1] > self.currentlength[1]):
            command_b[2] *= 1
        elif (self.newlength[1] < self.currentlength[1]):
            command_b[2] *= -1
        else:
            command_b[2]  = 0

        # Motor C
        # =======
        # Setting Angle
        command_c = self.command_buffer[2]
        if (self.newlength[2] > self.currentlength[2]):
            command_c[2] *= -1
        elif (self.newlength[2] < self.currentlength[2]):
            command_c[2] *= 1
        else:
            command_c[2]  = 0

        # Motor D
        # =======
        # Setting Angle
        command_d = self.command_buffer[3]
        if (self.newlength[3] > self.currentlength[3]):
            command_d[2] *= 1
        elif (self.newlength[3] < self.currentlength[3]):
            command_d[2] *= -1
        else:
            command_d[2]  = 0

        self.command_buffer = [command_a, command_b, command_c, command_d]
        self.move_ee_RPM()

    def move_ee_RPM(self):
        
        def run_motor(motor, speed, degrees):
            motor.on_for_degrees(SpeedRPM(speed), degrees)
        
        print("Status: End Effector Movement Started")
        threads = []
        for command in self.command_buffer:
            thread = Thread(target=run_motor, args=(command[0], command[1], command[2]))
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join()
        print("Status: End Effector Movement Completed")
        
        # Update Current EE Position
        self.o = self.new
        self.points = self.new_points
        self.currentlength = self.newlength

    # Visual Servoing Focused Functions [Not Used]

    def move_ee_vs(self):
        
        def run_motor(motor, speed, degrees):
            motor.on_for_degrees(SpeedPercent(speed), degrees)
        threads = []
        for command in self.command_buffer:
            thread = Thread(target=run_motor, args=(command[0], command[1], command[2]))
            thread.start()
            threads.append(thread)
        for thread in threads:
            thread.join()
        # Update Current EE Position
        self.o = self.new
        self.points = self.new_points
        self.currentlength = self.newlength

    def updating_current_ee_position(self,coords):
        
        current_ee_coords = coords
        
        # Updating End Effector Centre and Frame Coordinates Variables (Assuming EE at Centre of System)
        self.o = current_ee_coords
        p = [self.o[0] - self.ee_frame_width/2 , self.o[1] + self.ee_frame_length/2]
        q = [self.o[0] + self.ee_frame_width/2 , self.o[1] + self.ee_frame_length/2]
        r = [self.o[0] + self.ee_frame_width/2 , self.o[1] - self.ee_frame_length/2]
        s = [self.o[0] - self.ee_frame_width/2 , self.o[1] - self.ee_frame_length/2]
        self.points = [p,q,r,s] # End Effector Frame Coordinates [[xP,yP],[xQ,yQ],[xR,yR],[xS,yS]]
        time.sleep(0.5)

    def visual_servoing_movement(self, list):

        # list = [[current_ee_x,current_ee_y] , [current_target_x,current_target_y]]
        current_ee_coords = list[0]
        current_target_coords = list[0]

        self.updating_current_ee_position(current_ee_coords)
        self.update_target_using_coords(current_target_coords)
        self.update_command_buffer()
        self.move_ee_vs()