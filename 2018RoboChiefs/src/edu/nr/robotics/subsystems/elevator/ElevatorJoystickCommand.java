package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.OI;
import edu.nr.robotics.subsystems.climber.Climber;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class ElevatorJoystickCommand extends JoystickCommand {
	
	private static final double MIN_ELEV_JOYSTICK_PERCENT = 0;
	private static final double MAX_ELEV_JOYSTICK_PERCENT = 0.41;
		
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
			if (Elevator.getInstance().getPosition().lessThan(new Distance(3, Distance.Unit.INCH))) {
				Elevator.getInstance().setMotorPercentRaw(0);
			}
			else if (!EnabledSensors.elevatorSensor.get()) {
				Elevator.getInstance().setMotorPercentRaw(Elevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP);
			} else {
				Elevator.getInstance().setMotorPercentRaw(Elevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_DOWN);
			}
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
		return Elevator.getInstance().getCurrentCommand() == null || OI.getInstance().isElevatorNonZero();
	}

	@Override
	protected long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}

}
