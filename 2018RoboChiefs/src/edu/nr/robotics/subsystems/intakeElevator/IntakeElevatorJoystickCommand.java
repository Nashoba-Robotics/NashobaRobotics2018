package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.OI;
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
		if (!OI.getInstance().isIntakeElevatorNonZero()) {
			if (!EnabledSensors.intakeSensorLeft.get() && !EnabledSensors.intakeSensorRight.get())
				IntakeElevator.getInstance().setMotorSpeedRaw(0.129);
			else {
				IntakeElevator.getInstance().setMotorSpeedRaw(0.09);
			}
		} else if (OI.getInstance().getIntakeElevatorJoystickValue() > 0) {
			double motorPercent = OI.getInstance().getIntakeElevatorJoystickValue();
			IntakeElevator.getInstance().setMotorSpeedPercent(motorPercent);
		} else if (OI.getInstance().getIntakeElevatorJoystickValue() < 0) {
			double motorPercent = OI.getInstance().getIntakeElevatorJoystickValue();
			IntakeElevator.getInstance().setMotorSpeedPercent(motorPercent);
		}
	}
	
	@Override
	protected boolean shouldSwitchToJoystick() {
		return IntakeElevator.getInstance().getCurrentCommand() == null || OI.getInstance().isIntakeElevatorNonZero();
	}

	@Override
	protected long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}
	
}
