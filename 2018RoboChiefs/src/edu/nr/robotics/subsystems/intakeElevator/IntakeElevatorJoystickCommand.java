package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class IntakeElevatorJoystickCommand extends JoystickCommand {
	
	/**
	 * Takes intake elevator joystick percent values and sets the elevator to those percents.
	 * If the joystick tells the intake elevator not to move, then the elevator holds position
	 */
	public IntakeElevatorJoystickCommand() {
		super(IntakeElevator.getInstance());
	}

	@Override
	protected void onStart() {
	}
	
	@Override
	protected void onExecute() {
		if (!EnabledSensors.intakeSensorLeft.get() && !EnabledSensors.intakeSensorRight.get()) {
			IntakeElevator.getInstance().setMotorSpeedPercent(IntakeElevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ELEVATOR_UP);
		} else {
			IntakeElevator.getInstance().setMotorSpeedPercent(IntakeElevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ELEVATOR_DOWN);
		}
	}
	
	@Override
	protected boolean shouldSwitchToJoystick() {
		return IntakeElevator.getInstance().getCurrentCommand() == null;
	}

	@Override
	protected long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}
	
}
