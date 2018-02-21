package edu.nr.robotics.subsystems.elevatorShooter;

import edu.nr.lib.commandbased.NRCommand;

public class ElevatorShooterStopCommand extends NRCommand {

	/**
	 * Sets motor speed to 0
	 */
	public ElevatorShooterStopCommand() {
		super(ElevatorShooter.getInstance());
	}
	
	@Override
	protected void onStart() {
		ElevatorShooter.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}
	
}
