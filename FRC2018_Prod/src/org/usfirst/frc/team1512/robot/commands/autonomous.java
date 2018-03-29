package org.usfirst.frc.team1512.robot.commands;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class autonomous extends CommandGroup {
 public autonomous(int i) {
	switch (i) {
	case 0: driveFromLeft(1.0, 0.5);
	case 1: driveFromRight();
	case 2: driveFromCenter();
	case 3: scoreFromLeft();
	case 4: scoreFromRight();
	case 5: scoreFromCenter();
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
}