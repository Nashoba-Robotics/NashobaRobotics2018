package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.robotics.OI;

public class ElevatorJoystickCommand extends JoystickCommand {
	
	private static final double MIN_ELEV_JOYSTICK_PERCENT = 0.25;
	private static final double MAX_ELEV_JOYSTICK_PERCENT = 0.35;
	
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
			
		if (!OI.getInstance().isElevatorNonZero()) {
			Elevator.getInstance().setPosition(Elevator.getInstance().getPosition());
		} else if (OI.getInstance().getElevatorJoystickValue() > 0) {
			double motorPercent = OI.getInstance().getElevatorJoystickValue() * (MAX_ELEV_JOYSTICK_PERCENT - MIN_ELEV_JOYSTICK_PERCENT) + MIN_ELEV_JOYSTICK_PERCENT;
			Elevator.getInstance().setMotorSpeedPercent(motorPercent);
		} else if (OI.getInstance().getElevatorJoystickValue() < 0) {
			double motorPercent = OI.getInstance().getElevatorJoystickValue() * (MAX_ELEV_JOYSTICK_PERCENT - MIN_ELEV_JOYSTICK_PERCENT) - MIN_ELEV_JOYSTICK_PERCENT;
			Elevator.getInstance().setMotorSpeedPercent(motorPercent);
		}
	}
	
	@Override
	protected boolean shouldSwitchToJoystick() {
		return OI.getInstance().isElevatorNonZero();
	}

	@Override
	protected long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}

}
