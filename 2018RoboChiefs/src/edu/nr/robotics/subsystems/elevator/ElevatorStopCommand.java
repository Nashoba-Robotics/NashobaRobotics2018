package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;

public class ElevatorStopCommand extends NRCommand {

	/**
	 * Sets the elevator speed to 0
	 */
	public ElevatorStopCommand() {
	}
	
	@Override
	protected void onStart() {
		Elevator.getInstance().setMotorSpeedPercent(0);
	}
	
	@Override 
	protected boolean isFinishedNR() {
		return true;
	}
}
