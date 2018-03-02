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
	Counter counter2 = new Counter(RobotMap.limitSwitch2);
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public boolean isLowSwitchSet() {
		return counter2.get() > 0;
	}
	
	public boolean isTopSwitchSet() {
		return counter.get() > 0;
	}
	
	public void initializeCounter() {
		counter.reset();
	}
	public void moveUp() {
		RobotMap.thirdTalon.set(0.1);
	}
	public void moveDown() {
		RobotMap.thirdTalon.set(-0.1);
	}
	
	public void stop() {
		counter.reset();
		counter2.reset();
		RobotMap.thirdTalon.set(0.0);
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

