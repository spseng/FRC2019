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
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Spark;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot { 
	private  XboxController xbox;
	private WPI_TalonSRX FrontLeftMotor;
	private WPI_TalonSRX FrontRightMotor;
	

	@Override
	public void robotInit() {
		//m_leftStick = new Joystick(0);
		//m_rightStick = new Joystick(1);
		xbox = new XboxController(0);
		FrontLeftMotor = new WPI_TalonSRX(2);
		FrontRightMotor = new WPI_TalonSRX(4);
		
		
		
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
		
	}
	
}
