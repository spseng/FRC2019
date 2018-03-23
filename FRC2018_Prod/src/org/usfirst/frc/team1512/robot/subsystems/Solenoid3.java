package org.usfirst.frc.team1512.robot.subsystems;

import org.usfirst.frc.team1512.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Solenoid3 extends Subsystem {

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
	
	public void in() {
		RobotMap.solenoid3.set(DoubleSolenoid.Value.kForward);
	}
	
	public void out() {
		RobotMap.solenoid3.set(DoubleSolenoid.Value.kReverse);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}
