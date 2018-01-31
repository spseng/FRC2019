/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1512.robot;

import edu.wpi.first.wpilibj.GenericHID;
//import com.ctre.phoenix.
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot { 
	private  XboxController xbox;
	private WPI_TalonSRX FrontLeftMotor;
	private WPI_TalonSRX FrontRightMotor;
	private Potentiometer pot;
	private AnalogInput ai;
	private double degreesPot;

	@Override
	public void robotInit() {
		//m_leftStick = new Joystick(0);
		//m_rightStick = new Joystick(1);
		xbox = new XboxController(0);
		FrontLeftMotor = new WPI_TalonSRX(2);
		FrontRightMotor = new WPI_TalonSRX(4);
		ai = new AnalogInput(3);
		pot = new AnalogPotentiometer(ai, 360, 30);
		degreesPot = 0;
	}

	@Override
	public void teleopPeriodic() {
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
		
		degreesPot = pot.get();
		
		System.out.println(degreesPot);
		
	}
	
}
