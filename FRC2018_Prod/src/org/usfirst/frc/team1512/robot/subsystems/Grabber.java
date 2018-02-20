package org.usfirst.frc.team1512.robot.subsystems;

import org.usfirst.frc.team1512.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Grabber extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public void toggleCompressor() {
		boolean closedLoop = RobotMap.compressor.getClosedLoopControl();
		if(closedLoop) {
			RobotMap.compressor.setClosedLoopControl(false);
		}
		else {
			RobotMap.compressor.setClosedLoopControl(true);
		}
	}
	
	public void open() {
		RobotMap.grabber.set(false);
	}
	
	public void close() {
		RobotMap.grabber.set(true);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

