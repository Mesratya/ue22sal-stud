import rob1a_v01 as rob1a  # get robot simulator
import robot_control  # get robot control functions 
import sonar_filter # low pass filters
import numpy as np
import time

# the best approach is to place all your functions in robot_control.py
# however, if you need additionnal functions, you have to put them here
# ...

if __name__ == "__main__":
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl = robot_control.RobotControl() # create a robot controller

    # put here your code to solve the challenge
    speed = 60 # 40 ça marcjait nickel
    dist = 0.5
    ctrl.goLineOdometer(rb, dist, speed)
    angle = 90.0
    ctrl.inPlaceTurnRight(rb, angle)
    dist = 0.5
    ctrl.goLineOdometer(rb, dist, speed)
    angle = 90.0
    ctrl.inPlaceTurnLeft(rb, angle)
    dist = 1.5
    ctrl.goLineOdometer(rb, dist, speed)
    ctrl.inPlaceTurnRight(rb, angle)
    ctrl.followTheLeftWall(rb,0.5,speed)
    ctrl.inPlaceTurnRight(rb, angle)
    ctrl.followTheLeftWall(rb,0.5,40)

    speed = 40

    ctrl.inPlaceTurnRight(rb, angle)
    ctrl.goLineOdometer(rb,0.55, speed)
    ctrl.inPlaceTurnRight(rb, 22)

    speed = 20 # ne pas toucher à cette vitesse elle joue sur le temps de parcourt des murs en v !
    wall_time = 27
    ctrl.followTheRightWall(rb,0.5,speed,wall_time)
    ctrl.inPlaceTurnLeft(rb,20)
    ctrl.followTheRightWall(rb,0.5,speed,4)
    ctrl.inPlaceTurnLeft(rb,18)
    ctrl.followTheRightWall(rb,0.5,speed)


    #ctrl.goLineOdometer(rb,0.1,speed)
    ctrl.followTheLeftWall(rb,0.5,20)

    ctrl.inPlaceTurnRight(rb,90)
    ctrl.followTheLeftWall(rb,0.5,20,max_time=15)

    ctrl.inPlaceTurnRight(rb,18)

    wall_time = 26

    ctrl.followTheRightWall(rb, 0.5, speed,wall_time)
    ctrl.inPlaceTurnLeft(rb, 20)
    ctrl.followTheRightWall(rb, 0.5, speed, 4)
    ctrl.inPlaceTurnLeft(rb, 18)
    ctrl.followTheRightWall(rb, 0.5, speed)


    ctrl.followTheLeftWall(rb,0.5,15,0.5)



    

    # ajouter du piétinement pour être sur de scorer le checkpoint final !








    # safe end : stop the robot, then stop the simulator
    rb.stop()
    rb.log_file_off() # end log
    rb.full_end()
