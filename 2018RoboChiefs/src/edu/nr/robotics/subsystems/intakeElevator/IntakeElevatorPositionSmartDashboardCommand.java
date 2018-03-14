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
		boolean finished = IntakeElevator.getInstance().getVelocity().lessThan(IntakeElevator.PROFILE_STOP_SPEED_THRESHOLD)
		&& (IntakeElevator.getInstance().getPosition().sub(height)).abs().lessThan(IntakeElevator.PROFILE_END_POS_THRESHOLD_INTAKE_ELEVATOR);
		return finished;
	}
	
}
