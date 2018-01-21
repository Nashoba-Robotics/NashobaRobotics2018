package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class IntakeElevatorDeltaPositionSmartDashboardCommand extends NRCommand {

	private Distance initialPos;
	
	/**
	 * Sets the intake elevator position to the change specified
	 * @param deltaHeight: The change in height to go. Up is positive and down is negative
	 */
	public IntakeElevatorDeltaPositionSmartDashboardCommand() {
		super(IntakeElevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		this.initialPos = IntakeElevator.getInstance().getPosition();
		IntakeElevator.getInstance().setPosition(initialPos.add(IntakeElevator.profileDeltaPos));
	}
	
	@Override
	protected boolean isFinishedNR() {
		boolean finished = (IntakeElevator.getInstance().getHistoricalPosition(IntakeElevator.PROFILE_DELTA_TIME_THRESHOLD_INTAKE_ELEVATOR)
				.sub(IntakeElevator.getInstance().getPosition())).abs().lessThan(IntakeElevator.PROFILE_DELTA_POS_THRESHOLD_INTAKE_ELEVATOR)
				&& (IntakeElevator.getInstance().getHistoricalPosition(IntakeElevator.PROFILE_DELTA_TIME_THRESHOLD_INTAKE_ELEVATOR.mul(2))
						.sub(IntakeElevator.getInstance().getPosition())).abs().lessThan(IntakeElevator.PROFILE_DELTA_POS_THRESHOLD_INTAKE_ELEVATOR)
				&& (initialPos.add(IntakeElevator.profileDeltaPos).sub(IntakeElevator.getInstance().getPosition())).abs().lessThan(IntakeElevator.PROFILE_END_POS_THRESHOLD_INTAKE_ELEVATOR);
		return finished;
	}

}
