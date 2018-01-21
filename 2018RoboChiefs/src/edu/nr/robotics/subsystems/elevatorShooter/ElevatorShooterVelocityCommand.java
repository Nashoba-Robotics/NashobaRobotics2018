package edu.nr.robotics.subsystems.elevatorShooter;

import edu.nr.lib.commandbased.NRCommand;

public class ElevatorShooterVelocityCommand extends NRCommand {

	private double percent;
	
	public ElevatorShooterVelocityCommand(double percent) {
		this.percent = percent;
	}

	@Override
	protected void onStart() {
		ElevatorShooter.getInstance().setMotorSpeedPercent(percent);
	}

	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
