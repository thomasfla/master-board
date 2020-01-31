# coding: utf8

import argparse
import math
import os
import sys
from time import clock

import libmaster_board_sdk_pywrap as mbs
class GotoController():
    def __init__(self,robot_if,n_motors,dt):
        '''Control motor to reach a given position 'pos' in T seconds'''
        self.n_motors = n_motors
        self.robot_if = robot_if
        self.dt = dt  

        self.pos = n_motors * [0.]
        self.initial_position = n_motors * [0.]
        self.dt = dt

        self.READING_INITIAL_POSITION = 0
        self.CONTROLLING = 1
        
        self.state = n_motors * [self.READING_INITIAL_POSITION]
        self.control = n_motors * [0.]
        self.Kp = 5.0
        self.Kd = 0.015
        self.iq_sat = 5.0

        self.T = 1.0
        self.t = 0.0
    def ManageControl(self):
        self.t+=self.dt
        for motor in range(self.n_motors):
            if self.robot_if.GetMotor(motor).IsEnabled():
                #*** State machine ***
                if (self.state[motor] == self.READING_INITIAL_POSITION):
                # READING INITIAL POSITION
                    self.initial_position[motor] = self.robot_if.GetMotor(motor).GetPosition()
                    self.state[motor] = self.CONTROLLING

                elif (self.state[motor] == self.CONTROLLING):
                # POSITION CONTROL
                    if (self.t<self.T):
                        traj = self.initial_position[motor] + (self.pos[motor]-self.initial_position[motor])*0.5*(1-math.cos(2*math.pi*(0.5/self.T)*self.t))
                    else:
                        traj = self.pos[motor]
                    self.control[motor] = self.Kp*(traj - self.robot_if.GetMotor(motor).GetPosition() - self.Kd*self.robot_if.GetMotor(motor).GetVelocity())
                #*** END OF STATE MACHINE ***
                self.control[motor] = min(self.iq_sat, max(-self.iq_sat, self.control[motor]))
                self.robot_if.GetMotor(motor).SetCurrentReference(self.control[motor])
        return (self.t>self.T)

class CalibrationController():
    '''Control motors to find encoders index'''
    def __init__(self, robot_if, n_motors, dt):
        self.n_motors = n_motors
        self.robot_if = robot_if
        self.dt = dt   

        self.control = n_motors * [0.]
        self.initial_position = n_motors * [0.]

        self.READING_INITIAL_POSITION = 0
        self.SEARCHING_INDEX = 1
        self.CALIBRATION_DONE = 2
        self.state = n_motors * [self.READING_INITIAL_POSITION]

        self.Kp = 5.0
        self.Kd = 0.015
        self.iq_sat = 5.0
        self.all_motors_calibrated = False
        self.t = 0

    def ManageCalibration(self):
        self.t+=self.dt
        test_calibrated = True
        for motor in range(self.n_motors):
            if self.robot_if.GetMotor(motor).IsEnabled():
                #*** State machine for calibration ***
                if (self.state[motor] == self.READING_INITIAL_POSITION):
                # READING INITIAL POSITION
                    self.initial_position[motor] = self.robot_if.GetMotor(motor).GetPosition()
                    self.state[motor] = self.SEARCHING_INDEX

                elif (self.state[motor] == self.SEARCHING_INDEX):
                # POSITION CONTROL TO FIND INDEX
                    if (self.robot_if.GetMotor(motor).has_index_been_detected):
                        self.control[motor] = 0
                        self.state[motor] = self.CALIBRATION_DONE
                    else:
                        T=2.0
                        if (self.t<T/2.0):
                            calib_traj = self.initial_position[motor]+math.pi*0.5*(1-math.cos(2*math.pi*(1./T)*self.t))
                        else:
                            calib_traj = self.initial_position[motor]+math.pi*math.cos(2*math.pi*(0.5/T)*(self.t-T/2.0))
                        self.control[motor] = self.Kp*(calib_traj - self.robot_if.GetMotor(motor).GetPosition() - self.Kd*self.robot_if.GetMotor(motor).GetVelocity())
                #*** END OF STATE MACHINE ***

                self.control[motor] = min(self.iq_sat, max(-self.iq_sat, self.control[motor]))
                self.robot_if.GetMotor(motor).SetCurrentReference(self.control[motor])
            if (self.state[motor] != self.CALIBRATION_DONE):
                test_calibrated = False
        self.all_motors_calibrated = test_calibrated
        return self.all_motors_calibrated

def example_script(name_interface):

    N_SLAVES = 6  #  Maximum number of controled drivers
    N_SLAVES_CONTROLED = 1  # Current number of controled drivers

    cpt = 0  # Iteration counter
    dt = 0.001  #  Time step
    state = 0  # State of the system (ready (1) or not (0))

    print("-- Start of example script --")

    os.nice(-20)  #  Set the process to highest priority (from -20 highest to +20 lowest)

    robot_if = mbs.MasterBoardInterface(name_interface)
    robot_if.Init()  # Initialization of the interface between the computer and the master board
    for i in range(N_SLAVES_CONTROLED):  #  We enable each controler driver and its two associated motors
        robot_if.GetDriver(i).motor1.SetCurrentReference(0)
        robot_if.GetDriver(i).motor2.SetCurrentReference(0)
        robot_if.GetDriver(i).motor1.Enable()
        robot_if.GetDriver(i).motor2.Enable()
        robot_if.GetDriver(i).motor1.enable_index_offset_compensation = True #Important for calibration of encoders
        robot_if.GetDriver(i).motor2.enable_index_offset_compensation = True
        robot_if.GetDriver(i).EnablePositionRolloverError()
        robot_if.GetDriver(i).SetTimeout(5)
        robot_if.GetDriver(i).Enable()

    robot_if.GetDriver(0).motor1.SetPositionOffset(2.8)


    calibCtrl = CalibrationController(robot_if,N_SLAVES_CONTROLED * 2,dt)
    gotoCtrl = GotoController(robot_if,N_SLAVES_CONTROLED * 2,dt)
    last = clock()

    


    while ((not robot_if.IsTimeout()) and (clock() < 20)):  # Stop after 15 seconds (around 5 seconds are used at the start for calibration)
        if ((clock() - last) > dt):
            last = clock()
            cpt += 1
            robot_if.ParseSensorData()  # Read sensor data sent by the masterboard
            if (state == 0):  #  If the system is not ready
                state = 1
                for i in range(N_SLAVES_CONTROLED * 2):  # Check if all motors are enabled and ready
                    if not (robot_if.GetMotor(i).IsEnabled() and robot_if.GetMotor(i).IsReady()):
                        state = 0
            elif (state == 1):  # calibration    
                if calibCtrl.ManageCalibration():
                    state = 2
            elif (state==2):
                gotoCtrl.ManageControl()

            if ((cpt % 100) == 0):  # Display state of the system once every 100 iterations of the main loop
                print(chr(27) + "[2J")
                # To read IMU data in Python use robot_if.imu_data_accelerometer(i), robot_if.imu_data_gyroscope(i)
                # or robot_if.imu_data_attitude(i) with i = 0, 1 or 2
                robot_if.PrintIMU()
                robot_if.PrintADC()
                robot_if.PrintMotors()
                robot_if.PrintMotorDrivers()
                print(state)
                sys.stdout.flush()  # for Python 2, use print( .... , flush=True) for Python 3

            robot_if.SendCommand()  # Send the reference currents to the master board

    robot_if.Stop()  # Shut down the interface between the computer and the master board

    if robot_if.IsTimeout():
        print("Masterboard timeout detected.")
        print("Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")

    print("-- End of example script --")
    from IPython import embed
    embed()

def main():
    parser = argparse.ArgumentParser(description='Example masterboard use in python.')
    parser.add_argument('-i',
                        '--interface',
                        required=True,
                        help='Name of the interface (use ifconfig in a terminal), for instance "enp1s0"')

    example_script(parser.parse_args().interface)


if __name__ == "__main__":
    main()
