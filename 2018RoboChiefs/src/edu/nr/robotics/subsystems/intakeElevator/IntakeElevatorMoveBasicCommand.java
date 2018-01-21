package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.Elevator;

public class IntakeElevatorMoveBasicCommand extends NRCommand {


	private Distance height;
	private double percent;
	private Distance initialPos;

	public IntakeElevatorMoveBasicCommand(Distance height, double percent) {
		super(IntakeElevator.getInstance());
		this.height = height;
		this.percent = percent;
	}

	@Override
	protected void onStart() {
		initialPos = IntakeElevator.getInstance().getPosition();
		IntakeElevator.getInstance().setMotorSpeedPercent(Math.abs(percent) * height.signum());
	}

	@Override
	protected boolean isFinishedNR() {
		return (IntakeElevator.getInstance().getPosition().sub(initialPos)).abs()
				.lessThan(IntakeElevator.PROFILE_DELTA_POS_THRESHOLD_INTAKE_ELEVATOR);
	}
	
}
