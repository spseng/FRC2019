package org.usfirst.frc.team1512.robot.subsystems;

import org.usfirst.frc.team1512.robot.RobotMap;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator extends Subsystem {

	// catches fast limit switch changes
	Counter counter = new Counter(RobotMap.limitSwitch1);
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public boolean isSwitchSet() {
		return counter.get() > 0;
	}
	
	public void initializeCounter() {
		counter.reset();
	}
	public void moveUp() {
		RobotMap.firstSpark.set(0.1);
	}
	public void moveDown() {
		RobotMap.firstSpark.set(-0.1);
	}
	
	public void stop() {
		RobotMap.firstSpark.set(0.0);
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

