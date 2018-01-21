package edu.nr.robotics;

import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements SmartDashboardSource {
	
	private static OI singleton;
	
	private static final double JOYSTICK_DEAD_ZONE = 0.15;
	private static final int JOYSTICK_LEFT = 0;
	private static final int JOYSTICK_RIGHT = 1;
	
	public static final double DRIVE_SPEED_MULTIPLIER = 1;
	
	private final Joystick stickLeft;
	private final Joystick stickRight;
	
	public SendableChooser<Drive.DriveMode> DriveMode = new SendableChooser<>();
	
public void smartDashboardInfo() {
		
	}
		
	private OI() {
		
		stickLeft = new Joystick(JOYSTICK_LEFT);
		stickRight = new Joystick(JOYSTICK_RIGHT);
		
		initDriveLeft();
		initDriveRight();
		initDriveH();
		
		driveModeChooserInit();
		
		SmartDashboardSource.sources.add(this);
		
	}
	
	private void initDriveRight() {
	}

	private void initDriveLeft() {	
	}
	
	private void initDriveH() {
	}
	
	private void driveModeChooserInit() {
		DriveMode.addDefault("Arcade Drive", Drive.DriveMode.arcadeDrive);
		DriveMode.addObject("Tank Drive", Drive.DriveMode.tankDrive);
		DriveMode.addObject("Arcade With NegInertia", Drive.DriveMode.arcadeNegInertia);
		//DriveMode.addObject("Field Centric Drive", Drive.DriveMode.fieldCentric);
		
		SmartDashboard.putData("Drive Mode", DriveMode);
	}
	
	public static OI getInstance() {
		init();
		return singleton;
	}
	
	public double getDriveSpeedMultiplier() {
		return DRIVE_SPEED_MULTIPLIER;
	}
	
	public static void init() {
		if (singleton == null) {
				singleton = new OI();
		}
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
	
	public double getArcadeMoveValue() {
		return snapDriveJoysticks(-stickLeft.getY());
	}

	public double getArcadeTurnValue() {
		return -snapDriveJoysticks(stickRight.getX());
	}
	
	public double getArcadeHValue() {
		return snapDriveJoysticks(stickLeft.getX());
	}

	public double getTankLeftValue() {
		return snapDriveJoysticks(-stickLeft.getY());
	}

	public double getTankRightValue() {
		return snapDriveJoysticks(stickRight.getY());
	}
	
	public double getTankHValue() {
		return snapDriveJoysticks(stickLeft.getX());
	}
	
	public boolean isJoystickNonZero() {
		return ((Math.abs(stickRight.getX()) > JOYSTICK_DEAD_ZONE) || (Math.abs(stickLeft.getX()) > JOYSTICK_DEAD_ZONE) || (Math.abs(stickRight.getY()) > JOYSTICK_DEAD_ZONE) || (Math.abs(stickLeft.getY()) > JOYSTICK_DEAD_ZONE));
	}
}
