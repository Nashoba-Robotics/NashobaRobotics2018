package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.OI;
import edu.nr.robotics.Robot;
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
		
		IntakeElevator.getInstance().setMotorPercentRaw(0);
		
		if (!OI.getInstance().isIntakeElevatorNonZero()) {
			if (IntakeElevator.intakeFolded) {
				IntakeElevator.getInstance().setMotorPercentRaw(IntakeElevator.INTAKE_FOLD_HOLD_PERCENT);
			}
			else if (IntakeElevator.getInstance().getPosition().lessThan(new Distance(1, Distance.Unit.INCH))) {
			//if (IntakeElevator.getInstance().getPosition().lessThan(IntakeElevator.PORTAL_HEIGHT.mul(0.5))) {
				IntakeElevator.getInstance().setMotorPercentRaw(0);
			}
			else if (!EnabledSensors.intakeSensorLeft.get() && !EnabledSensors.intakeSensorRight.get())
				IntakeElevator.getInstance().setMotorPercentRaw(IntakeElevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ELEVATOR_UP);
			else {
				IntakeElevator.getInstance().setMotorPercentRaw(IntakeElevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ELEVATOR_DOWN);
			}
		} else if (OI.getInstance().getIntakeElevatorJoystickValue() > 0) {
			if (!IntakeElevator.intakeFolded) {
				IntakeElevator.intakeFolded = false;
			}
			double motorPercent = OI.getInstance().getIntakeElevatorJoystickValue();
			IntakeElevator.getInstance().setMotorSpeedPercent(motorPercent);
		} else if (OI.getInstance().getIntakeElevatorJoystickValue() < 0) {
			IntakeElevator.intakeFolded = false;
			double motorPercent = OI.getInstance().getIntakeElevatorJoystickValue();
			IntakeElevator.getInstance().setMotorSpeedPercent(motorPercent);
		}
	}
	
	@Override
	protected boolean shouldSwitchToJoystick() {
		return !Robot.getInstance().isAutonomous() && (IntakeElevator.getInstance().getCurrentCommand() == null || OI.getInstance().isIntakeElevatorNonZero());
	}

	@Override
	protected long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}
	
}
