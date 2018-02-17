/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1512.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;
	
	//TalonID as set in the live window
	public static WPI_TalonSRX firstTalon = new WPI_TalonSRX(2);
	public static WPI_TalonSRX secondTalon = new WPI_TalonSRX(4);
	public static WPI_TalonSRX thirdTalon = new WPI_TalonSRX(5);
	public static WPI_TalonSRX fourthTalon = new WPI_TalonSRX(6);
	
	//Joysticks setup
	public static Joystick leftJoystick = new Joystick(0);
	public static Joystick rightJoystick = new Joystick(1);
	public static XboxController xboxController = new XboxController(2);
	
	//Sensors setup
	static AnalogInput ai1 = new AnalogInput(1);
	public static AnalogPotentiometer pot1 = new AnalogPotentiometer(ai1, 360, 30);

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
