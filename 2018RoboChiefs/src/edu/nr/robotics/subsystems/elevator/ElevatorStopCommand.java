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
		Elevator.getInstance().disable();
	}
	
	@Override 
	protected boolean isFinishedNR() {
		return true;
	}
}
