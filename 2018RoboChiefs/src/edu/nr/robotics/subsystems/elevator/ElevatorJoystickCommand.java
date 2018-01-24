package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.robotics.OI;

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
	protected void onExecute() {
		
	}
	
	@Override
	protected boolean shouldSwitchToJoystick() {
		return false;
	}

	@Override
	protected long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}

}
