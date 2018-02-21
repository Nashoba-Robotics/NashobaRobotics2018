package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class IntakeElevatorPositionSmartDashboardCommand extends NRCommand {

private Distance height;
	
	public IntakeElevatorPositionSmartDashboardCommand() {
		super(IntakeElevator.getInstance());
		this.height = IntakeElevator.profileDeltaPos;
	}
	
	@Override
	protected void onStart() {
		IntakeElevator.getInstance().setPosition(height);
	}
	
	@Override
	protected boolean isFinishedNR() {
		boolean finished = (IntakeElevator.getInstance().getHistoricalPosition(IntakeElevator.PROFILE_DELTA_TIME_THRESHOLD_INTAKE_ELEVATOR)
		.sub(IntakeElevator.getInstance().getPosition())).abs().lessThan(IntakeElevator.PROFILE_DELTA_POS_THRESHOLD_INTAKE_ELEVATOR)
		&& (IntakeElevator.getInstance().getHistoricalPosition(IntakeElevator.PROFILE_DELTA_TIME_THRESHOLD_INTAKE_ELEVATOR.mul(2))
				.sub(IntakeElevator.getInstance().getPosition())).abs().lessThan(IntakeElevator.PROFILE_DELTA_POS_THRESHOLD_INTAKE_ELEVATOR)
		&& (IntakeElevator.getInstance().getPosition().sub(height)).abs().lessThan(IntakeElevator.PROFILE_END_POS_THRESHOLD_INTAKE_ELEVATOR);
		return finished;
	}
	
}
