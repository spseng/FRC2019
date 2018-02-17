package org.usfirst.frc.team1512.robot.subsystems;

import org.usfirst.frc.team1512.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {
	
	private StringBuilder _sb = new StringBuilder();
	TalonSRX talon = RobotMap.firstTalon;
	TalonSRX right = RobotMap.secondTalon;
	
	public DriveTrain() {

	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	
	public String testTalonSensors() {
		_sb.append("\tpos:");
		_sb.append(talon.getSelectedSensorPosition(0));
		_sb.append("u"); /* units */
		return _sb.toString();
	}
	
	/**
	 * All functions that you can test motor with:
	 * double currentAmps = talon.getOutputCurrent();
	 * double outputV = talon.getMotorOutputVoltage();
	 * double busV = talon.getBusVoltage();
	 * double outputPerc = talon.getMotorOutputPercent();
	 * int quadPos = talon.getSensorCollection().getQuadraturePosition();
	 * int quadVel = talon.getSensorCollection().getQuadratureVelocity();
	 * int analogPos = talon.getSensorCollection().getAnalogIn();
	 * int analogVel = talon.getSensorCollection().getAnalogInVel();
	 * int selectedSensorPos = talon.getSelectedSensorPosition(0); -- sensor selected for PID Loop 0
	 * int selectedSensorVel = talon.getSelectedSensorVelocity(0); -- sensor selected for PID Loop 0
	 * int closedLoopErr = talon.getClosedLoopError(0); -- sensor selected for PID Loop 0
	 * double closedLoopAccum = talon.getIntegralAccumulator(0); -- sensor selected for PID Loop 0
	 * double derivErr = talon.getErrorDerivative(0); -- sensor selected for PID Loop 0
	 */
	
}
