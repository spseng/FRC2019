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
//import edu.wpi.first.wpilibj.RobotDrive;  //RobotDrive deprecates to drive.DifferentialDrive
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

//Pneumatics
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
	private Joystick m_leftStick;	//driver station joysticks
	private Joystick m_rightStick;
	
	
	//motor controllers:
	private WPI_TalonSRX FrontLeftMotor;
	private WPI_TalonSRX FrontRightMotor;
//	private WPI_TalonSRX Shoulder;
//	private WPI_TalonSRX elbow;
	//private WPI_TalonSRX ArmMotor1;
	private WPI_TalonSRX Tower;   //moving tower winch up and down
	private WPI_TalonSRX GrabberLiftMotor;   //moving tower winch up and down
	private Spark MastLeft;
	private Spark MastRight;
	public static DigitalInput limitSwitch1 = new DigitalInput(3);
	public static DigitalInput limitSwitch2 = new DigitalInput(4);
	//Pneummatic controllers
	private Compressor compressor;
	//private DoubleSolenoid grabber;
	private Solenoid grabber;	
	//private Solenoid grabberLift;	
	private DoubleSolenoid grabberLift;
	//sensors:
	private Potentiometer pot1;
	private BuiltInAccelerometer Accel1; //this is the builtin 3axis accelerometer on the ADXRS450 port, connected to SPI port on roboRio
	private ADXRS450_Gyro Gyro1;	//this is the builtin gyro on the ADXRS450 port, connected to SPI port on roboRio
	
	//variables
	
		//grabber - pneumatic piece-grabbing system
			private boolean grabberclosed;
			private boolean grabberup;
			private boolean compressoron;
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
			
			private double towervalue;
			
		//RobotDrive
		DifferentialDrive myDrive;
	

	@Override
	public void robotInit() {
		towervalue =0.0;
		//driver station:
		xbox = new XboxController(2);			//value in parentheses is USB number
		m_leftStick = new Joystick(0);
		m_rightStick = new Joystick(1);
		CameraServer.getInstance().startAutomaticCapture();
		
		//robot motor controllers:
		FrontLeftMotor = new WPI_TalonSRX(2);	//value in parentheses is CANBUS device ID
		FrontRightMotor = new WPI_TalonSRX(4);
		//FrontLeftMotor.set(ControlMode.PercentOutput, m_leftStick.getY());
//		ArmMotor1 = new WPI_TalonSRX(5);
		Tower = new WPI_TalonSRX(5);  //moving tower up and down with winch
		GrabberLiftMotor = new WPI_TalonSRX(6);  //moving tower up and down with winch
		MastLeft = new Spark(0);
		MastRight = new Spark(1);
		
		
	
	//	elbow = new WPI_TalonSRX(6);
		
		//Robot drive system
		myDrive = new DifferentialDrive(FrontLeftMotor, FrontRightMotor);
		
		//pneumatics
		compressor = new Compressor(1); //PCM (pneumatics control module) is on CANBus at ID 1
		compressor.setClosedLoopControl(true); //this turns the control loop on, which means whenever pressure drops below 100psi compressor turns on
		//grabber = new DoubleSolenoid(1, 1,2);	//Double Solenoid port numbering on PCM begins at 0, first number is CANBus ID, second number is port number on PCM
		grabber = new Solenoid(1, 1);	//Solenoid port numbering on PCM begins at 0, first number is CANBus ID, second number is port number on PCM
//		grabberLift = new Solenoid(1, 2);	//Solenoid port numbering on PCM begins at 0, first number is CANBus ID, second number is port number on PCM
		grabberLift = new DoubleSolenoid(1, 2,3);	//Double Solenoid port numbering on PCM begins at 0, first number is CANBus ID, second number is port number on PCM
		compressoron=true; 				//records that compressor closed loop control is on
		grabberup=true;					// records that grabberLift starts with grabber in up position

			
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
			
			//grabber
			grabberclosed=true;

	}
	
	public void autonomousPeriodic() {
		myDrive.tankDrive(0.5, -0.5);
		Timer.delay(1.0);
		myDrive.tankDrive(0.0, 0.0);
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

		//TANK DRIVE SECTION - Uses left and right joystick
		//code to provide a "dead zone" for each joystick
		double leftvalue, rightvalue=0.0;
		if(m_leftStick.getY()>0.1 || m_leftStick.getY()<-0.1) 
		{
			leftvalue=m_leftStick.getY();
		}
		else 
		{
			leftvalue=0.0;
		}
		if(m_rightStick.getY()>0.1 || m_rightStick.getY()<-0.1) 
		{
			rightvalue=-1.0 * m_rightStick.getY();
		}
		else 
		{
			rightvalue=0.0;
		}
		myDrive.tankDrive(leftvalue, rightvalue);
		
		//Tower

		if(xbox.getRawAxis(1)>0.1 && limitSwitch1.get() == false)
		{
				System.out.println("inside if limitswitch1 test");
				towervalue=xbox.getRawAxis(1);
		}
		else if(xbox.getRawAxis(1)<-0.1 && limitSwitch2.get() == true) {
				System.out.println("inside if limitswitch2 test");
				towervalue=xbox.getRawAxis(1);
		}
		else
		{
			towervalue=0.0;
		}
		
		GrabberLiftMotor.set(towervalue);
		//grabberLift to run tower winch up and down
		

		//use talon and motor to raise/lower grabber
		double GrabberLiftMotorvalue =0.0;
		if(xbox.getRawAxis(5)>0.1 ||xbox.getRawAxis(5)<-0.1)
		{
			GrabberLiftMotorvalue=xbox.getRawAxis(5);
		}
		else
		{
			GrabberLiftMotorvalue=0.0;
		}
		
		Tower.set(GrabberLiftMotorvalue);
		 
		
		
		
		//MAST DRIVE SECTION - Uses xbox joystick
		//code to provide a "dead zone" for each joystick
		double mastvalue=0.0;
		if(xbox.getRawAxis(0)>0.1 || xbox.getRawAxis(0)<-0.1) 
		{
			mastvalue=xbox.getRawAxis(0);
		}
		else 
		{
			mastvalue=0.0;
		}
		//use mastvalue to drive TWO motors, connected to MastLeft and MastRight Spark Controllers
		MastLeft.set(mastvalue);
		MastRight.set(mastvalue*-1.0);  // right mast motor mounted opposite
		
		//run robot arm GRABBER
		if(xbox.getBumper(Hand.kRight) && grabberclosed==true) //if right bumper pressed and grabber is closed
		{
			
			while (xbox.getBumper(Hand.kRight))
			{
				//do nothing - wait until bumper is released before proceeding - stops double setting
			}
			
			grabber.set(false);	//open grabber
//			grabber.set(DoubleSolenoid.Value.kForward);	//open grabber
			grabberclosed=false;
		}
		else if(xbox.getBumper(Hand.kRight) && grabberclosed==false) //if right bumper pressed and grabber is open
		{
			while (xbox.getBumper(Hand.kRight))
			{
				//do nothing - wait until bumper is released before proceeding - stops double setting
			}

//			grabber.set(DoubleSolenoid.Value.kReverse);	//close grabber
			grabber.set(true);	//close grabber
			grabberclosed=true;			
		}
		/*
		else
		{
			grabber.set(DoubleSolenoid.Value.kOff);	//open grabber

		}
		*/

		//run robot arm GRABBERLIFT
		if(xbox.getBumper(Hand.kLeft) && grabberup==true) //if right bumper pressed and grabber is closed
		{
			
			while (xbox.getBumper(Hand.kLeft))
			{
				//do nothing - wait until bumper is released before proceeding - stops double setting
			}
			
//			grabberLift.set(false);	//open grabber
			grabberLift.set(DoubleSolenoid.Value.kForward);	//open grabberlift
			grabberup=false;
		}
		else if(xbox.getBumper(Hand.kLeft) && grabberup==false) //if right bumper pressed and grabber is open
		{
			while (xbox.getBumper(Hand.kLeft))
			{
				//do nothing - wait until bumper is released before proceeding - stops double setting
			}

			grabberLift.set(DoubleSolenoid.Value.kReverse);	//close grabberlift
//			grabberLift.set(true);	//close grabber
			grabberup=true;			
		}		
		else
		{
			grabberLift.set(DoubleSolenoid.Value.kOff);	//open grabber

		}
		

		
		
		//while debugging, allow the compressor to be turned off and on
		if(xbox.getAButton() && compressoron==true) //if left bumper pressed and compressor on
		{
			
			while (xbox.getAButton())
			{
				//do nothing - wait until bumper is released before proceeding - stops double setting
			}
			
			compressor.setClosedLoopControl(false);	//turn off compressor
			compressoron=false;
		}
		else if(xbox.getAButton() && compressoron==false) //if left bumper pressed and compressor off
		{
			while (xbox.getAButton())
			{
				//do nothing - wait until bumper is released before proceeding - stops double setting
			}
			
			compressor.setClosedLoopControl(true);	//turn on compressor
			compressoron=true;
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
		/*
		System.out.println("Accelerometer x reading:"+  (int) Accel1x);
		System.out.println("Accelerometer y reading:"+  (int) Accel1y);
		System.out.println("Accelerometer z reading:"+  (int) Accel1z);
		**/
		SmartDashboard.putBoolean("Limit Switch 1", limitSwitch1.get());
		SmartDashboard.putBoolean("Limit Switch 2", limitSwitch2.get());
		
		//Gyro1 display:
		SmartDashboard.putNumber("Gyro1 angle:",    Gyro1.getAngle());	//attempt to send info to driver station
		SmartDashboard.putNumber("Gyro1 rate:",   Gyro1.getRate());	//attempt to send info to driver station
		/*
		System.out.println("Gyro1 angle:"+   String.format("%.2f", Gyro1.getAngle()));
		System.out.println("Gyro1 rate:"+   String.format("%.2f", Gyro1.getRate()));
		**/
		//I'm trying to get gyro values to print with just 2 decimal points, but the putString doesn't show anything
		SmartDashboard.putString("Gyro1 angle:",   String.format("%.2f", Gyro1.getAngle()));	//attempt to send info to driver station
		SmartDashboard.putString("Gyro1 rate:",  String.format("%.2f", Gyro1.getRate()));	//attempt to send info to driver station
		
		// test setting xbox buttons to move arms based on potentiometer settings
		//low scale
/*  Trying to switch to tower raise/lower instead of arm, so this section is commented out:
  
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
		
		//end of commented-out section
*/

	}
		
}
	

