import numpy as np
import time
import random
import sonar_filter

class RobotControl:
    def __init__(self):
        self.distBetweenWheels = 0.12
        self.nTicksPerRevol = 720
        self.wheelDiameter = 0.06

    def testMove(self,rb,speedLeft,speedRight,duration):
        # forward motion
        rb.set_speed(speedLeft,speedRight)
        loopIterTime = 0.050
        tStart = time.time()
        while time.time()-tStart < duration:
            time.sleep(loopIterTime) # wait 
        # stop the robot 
        rb.stop()

    def testInfiniteObstacle (self,rb):
        loopIterTime = 0.050  # duration of loop iteration 50 ms
        legTimeMax = 5.0 # always turn after 5s (even if no obsctacle)
        distObstacle = 0.3 # stops and change direction if obstacle 
                           # at less than 30 cm
        while True:  # infinite loop : stop by typing ctrl-C
            rb.set_speed(90,90)
            tStartLeg = time.time()
            while True:
                t0 = time.time()
                if time.time()-tStartLeg >= legTimeMax:
                    break
                distFront = rb.get_sonar("front")
                #print ("distFront :",distFront)
                if distFront != 0.0 and distFront < distObstacle:
                    break
                t1 = time.time()
                dt = loopIterTime - (t1-t0)
                if dt>0:
                    time.sleep(dt) # wait for end of iteration
            # in the case the robot is trapped in front of a wall
            # go back at speed 40 for 0.5 seconds 
            rb.set_speed(-40,-40)
            time.sleep (0.5)
            # set random orientation by setting rotation duration
            # minimum time is 0.2 seconds, max is 1.5 seconds
            rotationTime = 0.2+1.3*random.random()
            rotationDirection = random.random()
            if rotationDirection < 0.5:
                self.testMove(rb,40,-40,rotationTime)
            else:
                self.testMove(rb,-40,40,rotationTime)

    def inPlaceTurnRight(self, rb, ang):   # ang in degrees  
        setPoint =  (self.nTicksPerRevol/360) * (ang*self.distBetweenWheels/self.wheelDiameter)
        loopIterationTime = 0.05
        odoLeft, odoRight = rb.get_odometers()
        while True :
            t0 = time.time()
            nTicksDone = rb.get_odometers()
            nTicksDone = nTicksDone[0] - odoLeft
            ControlError = setPoint-nTicksDone

            if abs(ControlError) < 2.5 :
                rb.stop()
                break
            rb.set_speed(5,-5)
            execTime = time.time() - t0
            deltaTime = loopIterationTime - execTime
            if deltaTime > 0 :
                time.sleep(deltaTime)



    def inPlaceTurnLeft(self, rb, ang):  # ang in degrees
        setPoint = (self.nTicksPerRevol / 360) * (ang * self.distBetweenWheels / self.wheelDiameter)
        loopIterationTime = 0.05
        odoLeft, odoRight = rb.get_odometers()
        while True:
            t0 = time.time()
            nTicksDone = rb.get_odometers()
            nTicksDone = nTicksDone[1] - odoRight
            ControlError = setPoint - nTicksDone
            if abs(ControlError) < 2.5:
                rb.stop()
                break
            rb.set_speed(-5, 5) # -5 et 5 ça marche bien
            execTime = time.time() - t0
            deltaTime = loopIterationTime - execTime
            if deltaTime > 0:
                time.sleep(deltaTime)

    def goLineOdometer(self, rb, dist, speed):  
        # dist in meter
        # speed in % (0 to 100)
        setPoint = (self.nTicksPerRevol / np.pi) * (dist/self.wheelDiameter)
        loopIterationTime = 0.05
        odoLeft, odoRight = rb.get_odometers()
        while True:
            t0 = time.time()
            nTicksDone = rb.get_odometers()
            nTicksDone = nTicksDone[1] - odoRight
            #ControlError = setPoint - nTicksDone
            if nTicksDone > setPoint and dist > 0:
                rb.stop()
                break
            if nTicksDone < setPoint and dist < 0:
                rb.stop()
                break
            if dist > 0 :
                rb.set_speed(speed, speed)
            if dist < 0 :
                rb.set_speed(-speed, -speed)
            execTime = time.time() - t0
            deltaTime = loopIterationTime - execTime

            if deltaTime > 0:
                time.sleep(deltaTime)

    def followTheLeftWall(self,rb,dist,speed,dist_Obstacle = 0.5,max_time = 120):
        setPoint = dist
        nominalSpeed = speed
        derivOk = False
        fltFront = sonar_filter.SonarFilter()
        fltFront.set_iir_a(0.7)
        fltFront.reset_iir()
        fltLeft = sonar_filter.SonarFilter()
        fltLeft.set_iir_a(0.7)
        fltLeft.reset_iir()
        kp = 10 # 30 et 50 /10 et 1000 / 4 et 4.4
        kd = 1000
        deltaSpeedMax = 10
        distObstacle = dist_Obstacle
        loopIterationTime = 0.05
        global_t0 = time.time()
        while True:
            t0 = time.time()

            distWall_raw = rb.get_sonar("left")
            distWall = fltLeft.median_filter(distWall_raw)
            distWall = fltLeft.iir_filter(distWall)
            ControlError_Left = setPoint - distWall
            if derivOk :
                derivError = ControlError_Left - lastError
                deltaSpeed = kp * ControlError_Left + kd * derivError
            else :
                deltaSpeed = kp * ControlError_Left
            #print("deltaSpeed = {0}".format(deltaSpeed))
            if deltaSpeed > deltaSpeedMax:
                deltaSpeed = deltaSpeedMax
            if deltaSpeed < - deltaSpeedMax:
                deltaSpeed = - deltaSpeedMax

            rb.set_speed(nominalSpeed + deltaSpeed, nominalSpeed - deltaSpeed)
            lastError = ControlError_Left
            derivOk = True

            distFront_raw = rb.get_sonar("front")
            distFront = fltFront.median_filter(distFront_raw)
            distFront = fltFront.iir_filter(distFront)
            #print(distFront_raw,distFront,distObstacle)
            ControlError_Front = distObstacle - distFront
            global_time = time.time() - global_t0
            print(global_time)
            if (distFront_raw != 0.0 and ControlError_Front > 0) or (global_time > max_time)  :
                rb.stop()
                break

            execTime = time.time() - t0
            deltaTime = loopIterationTime - execTime

            if deltaTime > 0:
                time.sleep(deltaTime)


    def followTheRightWall(self,rb,dist,speed,max_time = 120):
        setPoint = dist
        nominalSpeed = speed
        derivOk = False
        fltFront = sonar_filter.SonarFilter()
        fltFront.set_iir_a(0.7)
        fltFront.reset_iir()
        fltRight = sonar_filter.SonarFilter()
        fltRight.set_iir_a(0.7)
        fltRight.reset_iir()
        kp = 10
        kd = 1000
        deltaSpeedMax = 10
        distObstacle = 0.5
        loopIterationTime = 0.05
        global_t0 = time.time()
        while True:
            t0 = time.time()

            distWall_raw = rb.get_sonar("right")
            distWall = fltRight.median_filter(distWall_raw)
            distWall = fltRight.iir_filter(distWall)
            ControlError_Right = setPoint - distWall
            if derivOk :
                derivError = ControlError_Right - lastError
                deltaSpeed = kp * ControlError_Right + kd * derivError
            else :
                deltaSpeed = kp * ControlError_Right
            #print("deltaSpeed = {0}".format(deltaSpeed))
            if deltaSpeed > deltaSpeedMax:
                deltaSpeed = deltaSpeedMax
            if deltaSpeed < - deltaSpeedMax:
                deltaSpeed = - deltaSpeedMax

            rb.set_speed(nominalSpeed - deltaSpeed, nominalSpeed + deltaSpeed)
            lastError = ControlError_Right
            derivOk = True

            distFront_raw = rb.get_sonar("front")
            distFront = fltFront.median_filter(distFront_raw)
            distFront = fltFront.iir_filter(distFront)
            #print(distFront_raw,distFront,distObstacle)
            ControlError_Front = distObstacle - distFront
            global_time = time.time() - global_t0
            print(global_time)
            if (distFront_raw != 0.0 and ControlError_Front > 0) or (distWall_raw == 0.0) or (global_time > max_time) :
                rb.stop()
                break

            execTime = time.time() - t0
            deltaTime = loopIterationTime - execTime

            if deltaTime > 0:
                time.sleep(deltaTime)
