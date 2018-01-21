package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;

public class IntakeElevatorJoystickCommand extends JoystickCommand {

	//TODO: Implement IntakeElevatorJoystickCommand, including holding position when joystick at 0
	
	/**
	 * Takes intake elevator joystick percent values and sets the elevator to those percents.
	 * If the joystick tells the intake elevator not to move, then the elevator holds position
	 */
	public IntakeElevatorJoystickCommand() {
		super(IntakeElevator.getInstance());
	}

	@Override
	protected boolean shouldSwitchToJoystick() {
		//TODO: Check when we should switch to elevator joystick
		return false;
	}

	@Override
	protected long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}
	
}
