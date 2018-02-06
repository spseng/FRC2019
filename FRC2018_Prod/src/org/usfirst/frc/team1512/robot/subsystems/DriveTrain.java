package org.usfirst.frc.team1512.robot.subsystems;
import org.usfirst.frc.team1512.robot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class DriveTrain extends PIDSubsystem {
	
	
	public DriveTrain(double p, double i, double d) {
		super("DriveTrain", p, i, d);
		setAbsoluteTolerance(0.05);
		getPIDController().setContinuous(false);
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	
	protected double returnPIDInput() {
		return RobotMap.pot1.get();
	}
	
	protected void usePIDOutput(double output) {
		RobotMap.firstTalon.pidWrite(output);
	}
}
