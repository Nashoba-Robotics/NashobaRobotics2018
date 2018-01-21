package edu.nr.robotics.subsystems.intakeRollers;

import edu.nr.lib.commandbased.NRCommand;

public class IntakeRollersVelocityCommand extends NRCommand {

	private double percent;
	
	public IntakeRollersVelocityCommand(double percent) {
		super(IntakeRollers.getInstance());
		this.percent = percent;
	}

	@Override
	protected void onStart() {
		IntakeRollers.getInstance().setMotorSpeedPercent(percent);
	}

	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
