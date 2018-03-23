package org.usfirst.frc.team1512.robot.subsystems;

import org.usfirst.frc.team1512.robot.RobotMap;
import org.usfirst.frc.team1512.robot.commands.LiftGrabber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
			SmartDashboard.putBoolean("compressor state (false is off): ", false);
		}
		else {
			RobotMap.compressor.setClosedLoopControl(true);
			SmartDashboard.putBoolean("compressor state (false is off): ", true);
		}
	}
	
	public void open() {
		RobotMap.grabber.set(DoubleSolenoid.Value.kForward);
		SmartDashboard.putString("Grabber is: ", "open");
	}
	
	public void close() {
		SmartDashboard.putString("Grabber is: ", "closed");
		RobotMap.grabber.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void moveGrabber(double speed) {
		if(speed<-0.5) {
			speed = -0.5;
		}
		RobotMap.thirdTalon.set(speed);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new LiftGrabber());
    }
}

