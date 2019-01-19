package org.usfirst.frc.team1512.robot.commands;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;

import org.usfirst.frc.team1512.robot.Robot;
import org.usfirst.frc.team1512.robot.RobotMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class autonomous extends CommandGroup {
	NetworkTableValue turn;
	NetworkTableInstance inst = NetworkTableInstance.getDefault();
	NetworkTable table = inst.getTable("datatable");
	double degree = 0;
	
	autonomous(){
		requires(Robot.driveTrain);	
	}
	
 public autonomous(int i) {
	switch (i) {
	case 0: driveFromLeft(1.0, 0.5);
	case 1: driveFromRight();
	case 2: driveFromCenter();
	case 3: scoreFromLeft();
	case 4: scoreFromRight();
	case 5: scoreFromCenter();
	case 6: autoAvoid();
	}
 }
 
 public void driveFromLeft(double time, double speed) {
	 addSequential(new DriveStraight(time, speed));
 }
 public void driveFromRight() {
	 addSequential(new DriveStraight(0.0, 0.0));
 }
 public void driveFromCenter() {
	 addSequential(new DriveStraight(0.0, 0.0));
 }
 public void scoreFromLeft() {
	 addSequential(new DriveStraight(0.0, 0.0));
 }
 public void scoreFromRight() {
	 addSequential(new DriveStraight(0.0, 0.0));
 }
 public void scoreFromCenter() {
	 addSequential(new DriveStraight(0.0, 0.0));
 }
 public void autoAvoid() {
 }
}