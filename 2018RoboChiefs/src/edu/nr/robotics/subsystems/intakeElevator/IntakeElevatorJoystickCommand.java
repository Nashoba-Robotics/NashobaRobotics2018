package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;

public class IntakeElevatorJoystickCommand extends JoystickCommand {
	
	/**
	 * Takes intake elevator joystick percent values and sets the elevator to those percents.
	 * If the joystick tells the intake elevator not to move, then the elevator holds position
	 */
	public IntakeElevatorJoystickCommand() {
		super(IntakeElevator.getInstance());
	}

	@Override
	protected void onExecute() {
		IntakeElevator.getInstance().setPosition(IntakeElevator.getInstance().getPosition());
	}
	
	@Override
	protected boolean shouldSwitchToJoystick() {
		return IntakeElevator.getInstance().getCurrentCommand() != null;
	}

	@Override
	protected long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}
	
}
