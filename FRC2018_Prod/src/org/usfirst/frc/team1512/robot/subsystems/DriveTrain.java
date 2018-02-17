package org.usfirst.frc.team1512.robot.subsystems;
import org.usfirst.frc.team1512.robot.RobotMap;
import org.usfirst.frc.team1512.robot.commands.DriveWithJoysticks;


import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {
	
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoysticks());
	}

	public void tankDrive(double leftSpeed, double rightSpeed) {
		// Drives the motors
		RobotMap.firstTalon.set(leftSpeed);
		RobotMap.secondTalon.set(rightSpeed);
		
	}

}
