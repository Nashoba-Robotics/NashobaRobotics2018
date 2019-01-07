package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;

public class IntakeElevatorVelocityCommand extends NRCommand {

	private double percent;
	
	public IntakeElevatorVelocityCommand(double percent) {
		super(IntakeElevator.getInstance());
		this.percent = percent;
	}
	
	@Override
	protected void onStart() {
		IntakeElevator.getInstance().setMotorSpeedPercent(percent);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}
}
