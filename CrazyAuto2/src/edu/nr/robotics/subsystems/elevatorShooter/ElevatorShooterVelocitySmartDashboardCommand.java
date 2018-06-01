package edu.nr.robotics.subsystems.elevatorShooter;

import edu.nr.lib.commandbased.NRCommand;

public class ElevatorShooterVelocitySmartDashboardCommand extends NRCommand {

	public ElevatorShooterVelocitySmartDashboardCommand() {
		super(ElevatorShooter.getInstance());
	}

	@Override
	protected void onStart() {
		ElevatorShooter.getInstance().setMotorSpeedPercent(ElevatorShooter.shootPercent);
	}

	@Override
	protected boolean isFinishedNR() {
		return false;
	}
}