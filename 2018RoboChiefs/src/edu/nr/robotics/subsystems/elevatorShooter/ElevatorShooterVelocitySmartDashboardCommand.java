package edu.nr.robotics.subsystems.elevatorShooter;

import edu.nr.lib.commandbased.NRCommand;

public class ElevatorShooterVelocitySmartDashboardCommand extends NRCommand {

	public ElevatorShooterVelocitySmartDashboardCommand() {
		super(ElevatorShooter.getInstance());
	}

	@Override
	protected void onStart() {
		ElevatorShooter.getInstance().setMotorSpeedPercent(ElevatorShooter.VEL_PERCENT_ELEVATOR_SHOOTER);
	}

	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}