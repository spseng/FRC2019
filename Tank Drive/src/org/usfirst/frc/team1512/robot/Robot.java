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

asddfklasdklhf
as
ddfadsf
asd
f
a

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot {
	private DifferentialDrive m_myRobot;
	private Joystick m_leftStick;
	private Joystick m_rightStick;
	private  XboxController xbox;
	private RobotDrive myDrive;
	private TalonSRX FrontLeftMotor;
	private TalonSRX FrontRightMotor;
	private TalonSRX RearLeftMotor;
	private TalonSRX RearRightMotor;

	@Override
	public void robotInit() {
		m_leftStick = new Joystick(0);
		m_rightStick = new Joystick(1);
		xbox = new XboxController(0);
		FrontLeftMotor = new WPI_TalonSRX(1);
		FrontRightMotor = new WPI_TalonSRX(2);
		RearLeftMotor = new WPI_TalonSRX(3);
		RearRightMotor = new WPI_TalonSRX(4);
		
		
		
	}

	@Override
	public void teleopPeriodic() {
		
		m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
		m_myRobot.tankDrive(xbox.getY(GenericHID.Hand.kLeft), xbox.getY(GenericHID.Hand.kRight));
		
		FrontLeftMotor.set(ControlMode.PercentOutput, m_leftStick.getY());
	}
	
}
