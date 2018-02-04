/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1512.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;

//import com.ctre.phoenix.
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.*;

/**
 * 2/3/2018: This is the version of the generic TankDrive program, adapted to our robot for testing purposes
 * 
 * 	In Ryan Mitchell's view, because we will be trying to run multiple processes at the same time during
 * 	the competition, we will eventuallly move our code to the "FRC2018_Prod" project, which is a
 * 	COMMAND-BASED Robotics program.  
 * 
 * 	The following Tank Drive project uses an intereative robot class.  We have set up 3 talons, an xbox,
 * 	and 1 potentiometer.
 * 
 * 	I am attempting to add some commands to display information to the smart dashboard, especially to 
 * 	display the potentiometer, the gyroscope, and the accelerometer
 */
public class Robot extends IterativeRobot { 
	//driver station devices:
	private  XboxController xbox;	//driver station xbox controller
	//private Joystick m_leftStick;	//driver station joysticks
	//private Joystick m_rightStick;
	
	//motor controllers:
	private WPI_TalonSRX FrontLeftMotor;
	private WPI_TalonSRX FrontRightMotor;
	private WPI_TalonSRX ArmMotor1;
	
	//sensors:
	private Potentiometer pot1;
	private BuiltInAccelerometer Accel1; //this is the builtin 3axis accelerometer on the ADXRS450 port, connected to SPI port on roboRio
	private ADXRS450_Gyro Gyro1;	//this is the builtin gyro on the ADXRS450 port, connected to SPI port on roboRio
	
	//variables
		//Potentiometers:
			private double p1lowAngle;
			private double p1middleAngle;
			private double p1highAngle;
			private AnalogInput ai3;
			private double degreesPot1;
	
		//Accelermoters:
			private double Accel1x;
			private double Accel1y;
			private double Accel1z;
	

	@Override
	public void robotInit() {
		//driver station:
		xbox = new XboxController(0);			//value in parentheses is USB number
		//m_leftStick = new Joystick(1);
		//m_rightStick = new Joystick(2);
		
		//robot motor controllers:
		FrontLeftMotor = new WPI_TalonSRX(2);	//value in parentheses is CANBUS device ID
		FrontRightMotor = new WPI_TalonSRX(4);
		ArmMotor1 = new WPI_TalonSRX(3);
		
		//sensors:
		//potentiometer 1, connected to analog-input 3
		ai3 = new AnalogInput(3);
		pot1 = new AnalogPotentiometer(ai3, 360, 30);
		
		//input accelerometer
		Accel1 = new BuiltInAccelerometer();
		
		//builtin gyroscope (ADXRS450)
		Gyro1 = new ADXRS450_Gyro();
		
		//Gyro1.initGyro();
		Gyro1.calibrate();
		


		//variables:
			//potentiometer 1
			p1lowAngle = 150;
			p1middleAngle = 200;
			p1highAngle = 250;
			degreesPot1 = 0;
			
			//accelerometer
			Accel1x=Accel1y=Accel1z=0;
	}

	@Override
	public void teleopPeriodic() {
		
		/*
		 * Gyroscope is working.  Therefore it is possible to write code for driving straight:
		 * 
		 * Find this code in screensteps by searching for "Gyros - Measuring rotation and controlling robot driving direction"
		 * 
		 * The variable kP is a proportional scaling constant to scale it for the speed of the robot drive.  This factor is
		 * 	called the proportional constant or loop gain. Increasing kP will cause the robot to correct more quickly.
		 *   (but too high and it will oscillate).  Decreasing the constant will cause it to correct more slowly.
		 *   This is known as proportional control - refer to PID control for more details.
		 *   
		 *   note that this example assumes there is a myrobot class set up to drive robot.
		 *   
		 *   gyro1.reset();	//set zero degree direction
		 *   while (condition for length of time to drive straight)
		 *   {
		 *   	double angle = gyro.getAngle();	//get current heading
		 *   	myrobot.drive(-1.0, -angle * kP;	//move towards heading of 0 degrees
		 *   	Timer.delay(0.004);
		 *   }
		 *   myrobot.drive(0.0, 0.0); //stop robot
		 * 
		 */
		
		//Drive robot, but allow a "dead zone" around zero point of joystick
		if(xbox.getRawAxis(5)>0.1 || xbox.getRawAxis(5)<-0.1) {
			FrontLeftMotor.set(-1.0 * xbox.getRawAxis(5));
		}
		else {
			FrontLeftMotor.set(0.0);
		}
		if(xbox.getRawAxis(1)>0.1 || xbox.getRawAxis(1)<-0.1) {
			FrontRightMotor.set(xbox.getRawAxis(1));
		}
		else {
			FrontRightMotor.set(0.0);
		}
		
		//test potentiometer 1
		degreesPot1 = pot1.get();
		//40-390
		
		
		//when printing values, I am CASTING them to the int type. 
		//	This gets rid of the values to the right of the decimal point
		
		System.out.println("Potentiometer 1 reading:" + (int) degreesPot1);
		SmartDashboard.putNumber("Potentiometer 1 reading:", (int) degreesPot1);	//attempt to send info to driver station
		
		//test accelerometer 1
		Accel1x=Accel1.getX();
		Accel1y=Accel1.getY();
		Accel1z=Accel1.getZ();
		SmartDashboard.putNumber("Accelerometer x reading:", (int) Accel1x);	//attempt to send info to driver station
		SmartDashboard.putNumber("Accelerometer y reading:", (int) Accel1y);	//attempt to send info to driver station
		SmartDashboard.putNumber("Accelerometer z reading:", (int) Accel1z);	//attempt to send info to driver station
		System.out.println("Accelerometer x reading:"+  (int) Accel1x);
		System.out.println("Accelerometer y reading:"+  (int) Accel1y);
		System.out.println("Accelerometer z reading:"+  (int) Accel1z);
		
		//Gyro1 display:
		SmartDashboard.putNumber("Gyro1 angle:",    Gyro1.getAngle());	//attempt to send info to driver station
		SmartDashboard.putNumber("Gyro1 rate:",   Gyro1.getRate());	//attempt to send info to driver station
		System.out.println("Gyro1 angle:"+   String.format("%.2f", Gyro1.getAngle()));
		System.out.println("Gyro1 rate:"+   String.format("%.2f", Gyro1.getRate()));
	
		//I'm trying to get gyro values to print with just 2 decimal points, but the putString doesn't show anything
		SmartDashboard.putString("Gyro1 angle:",   String.format("%.2f", Gyro1.getAngle()));	//attempt to send info to driver station
		SmartDashboard.putString("Gyro1 rate:",  String.format("%.2f", Gyro1.getRate()));	//attempt to send info to driver station
		
		// test setting xbox buttons to move arms based on potentiometer settings
		//low scale
		if(xbox.getAButtonPressed()) {
			if(pot1.get()<p1lowAngle-5) {
				ArmMotor1.set(0.2);
			} else if (pot1.get()>p1lowAngle+5) {
				ArmMotor1.set(-0.2);
			} else {
				ArmMotor1.set(0.0);
				
			}
		}
		
		
		//middle scale position
		if(xbox.getBButtonPressed()) {
			if(pot1.get()<p1middleAngle-5) {
				ArmMotor1.set(0.2);
			} else if (pot1.get()>p1middleAngle+5) {
				ArmMotor1.set(-0.2);
			} else {
				ArmMotor1.set(0.0);
				
			}
		}
		//high scale position
		if(xbox.getYButtonPressed()) {
			if(pot1.get()<p1highAngle-5) {
				ArmMotor1.set(0.2);
			} else if (pot1.get()>p1highAngle+5) {
				ArmMotor1.set(-0.2);
			} else {
				ArmMotor1.set(0.0);
			}
		}
		
		if(xbox.getXButtonPressed()) {
			p1lowAngle = pot1.get();
		}
		
	}
		
}
	

