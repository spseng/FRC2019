/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1512.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.*;

import org.usfirst.frc.team1512.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team1512.robot.commands.LiftGrabber;
import org.usfirst.frc.team1512.robot.commands.autonomous;
import org.usfirst.frc.team1512.robot.subsystems.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static DriveTrain driveTrain;
	public static Elevator elevator;
	public static Grabber grabber;
	public static OI m_oi;
	NetworkTableValue turn;
	NetworkTableInstance inst = NetworkTableInstance.getDefault();
	NetworkTable table = inst.getTable("datatable");
	double degree = 0;
	
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 * 
	 * Note that its ok to initialize the subsystems here because their commands won't be
	 * scheduled by the Scheduler until teleopPeriodic starts
	 */
	@Override
	public void robotInit() {
		
		
		CameraServer.getInstance().startAutomaticCapture();
		driveTrain = new DriveTrain();
		elevator = new Elevator();
		grabber = new Grabber();
		m_oi = new OI();
		RobotMap.compressor.setClosedLoopControl(true);
		RobotMap.Gyro1.calibrate();
		m_chooser.addDefault("Default autonomous", new autonomous(0));
		SmartDashboard.putData("Auto mode", m_chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();
		
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		while(turn == null) {
			turn = table.getEntry("turn").getValue();
			System.out.println("DATA NOT FOUND");
		}
		turn = table.getEntry("turn").getValue();
		degree = turn.getDouble();
		System.out.println("OUTPUT == "+degree);
		Robot.driveTrain.tankDrive(degree/5,degree/5);
		//Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		SmartDashboard.putBoolean("Limit Switch 1", RobotMap.limitSwitch1.get());
		SmartDashboard.putBoolean("Limit Switch 2", RobotMap.limitSwitch2.get());
		SmartDashboard.putNumber("Gyro1 angle:", RobotMap.Gyro1.getAngle());
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
