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
import edu.wpi.first.wpilibj.AnalogGyro;



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
	private BuiltInAccelerometer Accel1;
	private AnalogGyro Gyro1;
	
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
		
		System.out.println("Potentiometer 1 reading:" + degreesPot1);
		SmartDashboard.putNumber("Potentiometer 1 reading:", degreesPot1);	//attempt to send info to driver station
		
		//test accelerometer 1
		Accel1x=Accel1.getX();
		Accel1y=Accel1.getY();
		Accel1z=Accel1.getZ();
		SmartDashboard.putNumber("Accelerometer x reading:", Accel1x);	//attempt to send info to driver station
		SmartDashboard.putNumber("Accelerometer y reading:", Accel1y);	//attempt to send info to driver station
		SmartDashboard.putNumber("Accelerometer z reading:", Accel1z);	//attempt to send info to driver station
		System.out.println("Accelerometer x reading:"+  Accel1x);
		System.out.println("Accelerometer y reading:"+  Accel1y);
		System.out.println("Accelerometer z reading:"+  Accel1z);
		
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
	

