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
    speed = 40
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
    ctrl.followTheLeftWall(rb,0.5,speed)

    print("J'affronte le premier mur en V !")

    ctrl.inPlaceTurnRight(rb, angle)
    ctrl.goLineOdometer(rb,0.55, speed)
    ctrl.inPlaceTurnRight(rb, 25)

    speed = 20

    ctrl.followTheRightWall(rb,0.6,speed)
    print("fin du mur en V n°1, je tourne")
    ctrl.goLineOdometer(rb,0.1,speed)
    print("suivit à gauche")
    ctrl.followTheLeftWall(rb,0.5,speed)
    print("fin du suivit à gauche : rotation !")
    ctrl.inPlaceTurnRight(rb,70)
    print("J'affronte le 2nd mur en V ! , j'avance un peu")
    ctrl.goLineOdometer(rb,0.5,speed)
    print("Fin des préparatifs je suit ce fameux mur en V")
    ctrl.followTheRightWall(rb,0.6,speed)
    print("Fin du suivit, légère rotation vers la droite enclanchée")
    ctrl.inPlaceTurnRight(rb,10)
    print("Phase Finale suivit du mur gauche !")
    ctrl.followTheLeftWall(rb,0.5,speed)
    print("Fin !!!")








    # safe end : stop the robot, then stop the simulator
    rb.stop()
    rb.log_file_off() # end log
    rb.full_end()
