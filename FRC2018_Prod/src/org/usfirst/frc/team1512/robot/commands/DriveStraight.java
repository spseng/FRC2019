package org.usfirst.frc.team1512.robot.commands;

import org.usfirst.frc.team1512.robot.OI;
import org.usfirst.frc.team1512.robot.Robot;
import org.usfirst.frc.team1512.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraight extends Command {
	OI oi;
    public DriveStraight() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	oi = new OI();
    	Robot.driveTrain.stop();
    	RobotMap.Gyro1.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double reading = RobotMap.Gyro1.getAngle();
    	if (reading > 0.0) {
    		Robot.driveTrain.tankDrive(oi.getLeftSpeed() * 0.15, -0.1 * oi.getLeftSpeed());
    	}
    	else {
    		Robot.driveTrain.tankDrive(oi.getLeftSpeed() * 0.1, -0.15 * oi.getLeftSpeed());
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
