/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.nr.robotics;

import edu.nr.lib.commandbased.CancelAllCommand;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements SmartDashboardSource{
	
	private static final double JOYSTICK_DEAD_ZONE = 0.15;
	
	private static final int CANCEL_ALL_BUTTON_NUMBER = 1;
	
	private double driveSpeedMultiplier = 1;
	
	private static OI singleton;
	
	public static final Drive.DriveMode driveMode = Drive.DriveMode.cheesyDrive;
	
	private final Joystick driveLeft;
	private final Joystick driveRight;
	
	private final Joystick operatorLeft;
	private final Joystick operatorRight;
	
	private static final int STICK_LEFT = 0;
	private static final int STICK_RIGHT = 1;
	private static final int STICK_OPERATOR_LEFT = 2;
	private static final int STICK_OPERATOR_RIGHT = 3;
	
	private OI() {
		driveLeft = new Joystick(STICK_LEFT);
		driveRight = new Joystick(STICK_RIGHT);
		
		operatorLeft = new Joystick(STICK_OPERATOR_LEFT);
		operatorRight = new Joystick(STICK_OPERATOR_RIGHT);
		
		initDriveLeft();
		initDriveRight();
		
		initOperatorLeft();
		initOperatorRight();
		SmartDashboardSource.sources.add(this);
	}
	
	public void initDriveLeft() {
		
	}
	
	public void initDriveRight() {
		
	}
	
	public void initOperatorLeft() {
		
	}
	
	public void initOperatorRight() {
		new JoystickButton(operatorRight, CANCEL_ALL_BUTTON_NUMBER).whenPressed(new CancelAllCommand());	
	}
	
	public static OI getInstance() {
		return singleton;
	}
	
	public static void init() {
		if(singleton == null) {
			singleton = new OI();
		}
	}
	
	public double getArcadeMoveValue() {
		return -snapDriveJoysticks(driveLeft.getY());//* (driveLeft.getRawButton(DRIVE_REVERSE_BUTTON_NUMBER) ? 1 : -1);
	}

	public double getArcadeTurnValue() {
		return -snapDriveJoysticks(driveRight.getX());
	}
	
	public double getTankLeftValue() {
		return snapDriveJoysticks(driveLeft.getY());
	}

	public double getTankRightValue() {
		return snapDriveJoysticks(driveRight.getY());
	}
	
	public double getDriveLeftXValue() {
		return snapDriveJoysticks(driveLeft.getX());
	}

	public double getDriveLeftYValue() {
		return snapDriveJoysticks(driveLeft.getY());
	}
	
	public double getDriveRightXValue() {
		return snapDriveJoysticks(driveRight.getX());
	}

	public double getDriveRightYValue() {
		return snapDriveJoysticks(driveRight.getY());
	}
	
	
	public double getDriveSpeedMultiplier() {
		return driveSpeedMultiplier;
	}
	
	private static double snapDriveJoysticks(double value) {
		if (Math.abs(value) < JOYSTICK_DEAD_ZONE) {
			value = 0;
		} else if (value > 0) {
			value -= JOYSTICK_DEAD_ZONE;
		} else {
			value += JOYSTICK_DEAD_ZONE;
		}
		value /= 1 - JOYSTICK_DEAD_ZONE;
		return value;
	}
	
	public double getRawMove() {
		return driveLeft.getY();
	}
	
	public double getRawTurn() {
		return driveRight.getX();
	}
	
	private double getTurnAdjust() {
		return driveRight.getRawButton(2) ? 0.5 : 1;
	}
	
	public void smartDashboardInfo() {

	}
	
	public boolean isTankNonZero() {
		return getTankLeftValue() != 0 || getTankRightValue() != 0;
	}
	
	public boolean isArcadeNonZero() {
		return getArcadeMoveValue() != 0 || getArcadeTurnValue() != 0  ;
	}
	
	public boolean isDriveNonZero() {
		return getDriveLeftXValue() != 0 || getDriveRightXValue() != 0 || getDriveLeftYValue() != 0 || getDriveRightYValue() != 0;
	}
		
	public boolean shouldDumbDrive() {
		//TODO: Find how we want to switch to dumb drive
		return false;
	}
}
