package edu.nr.robotics;

import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements SmartDashboardSource {
	
	private static OI singleton;
	
	private static final double JOYSTICK_DEAD_ZONE = 0.15;
	private static final int JOYSTICK_LEFT = 0;
	private static final int JOYSTICK_RIGHT = 1;
	
	private final Joystick stickLeft;
	private final Joystick stickRight;
	
	@Override
	public void smartDashboardInfo() {
		// TODO Auto-generated method stub
		
	}
	
	private OI() {
		
		stickLeft = new Joystick(JOYSTICK_LEFT);
		stickRight = new Joystick(JOYSTICK_RIGHT);
		
		initDriveLeft();
		initDriveRight();
		
		SmartDashboardSource.sources.add(this);
		
	}

	private void initDriveRight() {
	}

	private void initDriveLeft() {	
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
		return snapDriveJoysticks(stickLeft.getY());
	}

	public double getArcadeTurnValue() {
		return -snapDriveJoysticks(stickRight.getX());
	}

	public double getTankLeftValue() {
		return snapDriveJoysticks(stickLeft.getY());
	}

	public double getTankRightValue() {
		return snapDriveJoysticks(stickRight.getY());
	}
	
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
}
