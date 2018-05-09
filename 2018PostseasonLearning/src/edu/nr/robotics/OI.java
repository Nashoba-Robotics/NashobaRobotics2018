/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.nr.robotics;

import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements SmartDashboardSource{

	private static OI singleton;
	
	private final Joystick driverLeft;
	private final Joystick driverRight;

	private final Joystick operatorLeft;
	private final Joystick operatorRight;


	private static final int JOYSTICK_DRIVE_LEFT_PORT = 0;
	private static final int JOYSTICK_DRIVE_RIGHT_PORT = 1;
	private static final int JOYSTICK_OPERATE_LEFT_PORT = 2;
	private static final int JOYSTICK_OPERATE_RIGHT_PORT = 3;
	
	private static final double JOYSTICK_DEAD_ZONE = 0.15;
	
	
	private OI() {
		driverLeft = new Joystick(JOYSTICK_DRIVE_LEFT_PORT);
		driverRight = new Joystick(JOYSTICK_DRIVE_RIGHT_PORT);
		operatorLeft = new Joystick(JOYSTICK_OPERATE_LEFT_PORT);
		operatorRight = new Joystick(JOYSTICK_OPERATE_RIGHT_PORT);
		
		initLeftDriverButton();
		initRightDriverButton();
		initLeftOperatorButton();
		initRightOperatorButton();
		
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
	
	private void initLeftDriverButton() {
			
	}
	
	private void initRightDriverButton() {
		
	}
	
	private void initLeftOperatorButton() {
		
	}
	
	private void initRightOperatorButton() {
		
	}
	
	public double getArcadeMoveValue() {
		return -snapDriveJoysticks(driverLeft.getY()) ;
	}

	public double getArcadeTurnValue() {
		return -snapDriveJoysticks(driverRight.getX()) ;
	}
	
	public static OI getInstance() {
		init();
		return singleton;
	}

	public static void init() {
		if (singleton == null) {
			singleton = new OI();
		}
	}
	
	@Override
	public void smartDashboardInfo() {
		// TODO Auto-generated method stub
		
	}
	
	
}
