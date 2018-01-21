package edu.nr.robotics.subsystems.intakeRollers;

import edu.nr.lib.commandbased.NRCommand;

public class IntakeRollersVelocitySmartDashboardCommand extends NRCommand {

	public IntakeRollersVelocitySmartDashboardCommand() {
		super(IntakeRollers.getInstance());
	}

	@Override
	protected void onStart() {
		IntakeRollers.getInstance().setMotorSpeedPercent(IntakeRollers.VEL_PERCENT_INTAKE_ROLLERS);
	}

	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}