package edu.nr.robotics.subsystems.intakeRollers;

import edu.nr.lib.commandbased.NRCommand;

public class IntakeRollersVelocityCommand extends NRCommand {

	private double percentHigh;
	private double percentLow;
	
	public IntakeRollersVelocityCommand(double percentHigh, double percentLow) {
		super(IntakeRollers.getInstance());
		this.percentHigh = percentHigh;
	}

	@Override
	protected void onStart() {
		IntakeRollers.getInstance().setMotorSpeedPercent(percentHigh, percentLow);
	}

	@Override
	protected boolean isFinishedNR() {
		return false;
	}
}
