package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class IntakeElevatorMoveBasicSmartDashboardCommand extends NRCommand {

	private Distance initialPos;

	public IntakeElevatorMoveBasicSmartDashboardCommand() {
		super(IntakeElevator.getInstance());
	}

	@Override
	protected void onStart() {
		initialPos = IntakeElevator.getInstance().getPosition();
		IntakeElevator.getInstance().setMotorSpeedPercent(Math.abs(IntakeElevator.PROFILE_VEL_PERCENT_INTAKE_ELEVATOR) * IntakeElevator.profileDeltaPos.signum());
	}

	@Override
	protected boolean isFinishedNR() {
		return (IntakeElevator.getInstance().getPosition().sub(initialPos.add(IntakeElevator.profileDeltaPos))).abs()
				.lessThan(IntakeElevator.PROFILE_DELTA_POS_THRESHOLD_INTAKE_ELEVATOR);
	}
	
}