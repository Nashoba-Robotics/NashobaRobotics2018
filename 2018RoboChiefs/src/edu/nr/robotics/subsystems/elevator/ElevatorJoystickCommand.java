package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.JoystickCommand;

public class ElevatorJoystickCommand extends JoystickCommand {

	//TODO: Implement ElevatorJoystickCommand, including holding position when joystick at 0
	
	/**
	 * Takes elevator joystick percent values and sets the elevator to those percents.
	 * If the joystick tells the elevator not to move, then the elevator holds position
	 */
	public ElevatorJoystickCommand() {
		super(Elevator.getInstance());
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
