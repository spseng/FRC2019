package org.usfirst.frc.team1512.robot.commands;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class autonomous extends CommandGroup {
 public autonomous(int i) {
	switch (i) {
	case 0: driveFromLeft();
	case 1: driveFromRight();
	case 2: driveFromCenter();
	case 3: scoreFromLeft();
	case 4: scoreFromRight();
	case 5: scoreFromCenter();
	}
 }
 
 public void driveFromLeft() {
	 addSequential(new DriveStraight());
 }
 public void driveFromRight() {
	 addSequential(new DriveStraight());
 }
 public void driveFromCenter() {
	 addSequential(new DriveStraight());
 }
 public void scoreFromLeft() {
	 addSequential(new DriveStraight());
 }
 public void scoreFromRight() {
	 addSequential(new DriveStraight());
 }
 public void scoreFromCenter() {
	 addSequential(new DriveStraight());
 }
}