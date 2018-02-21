package edu.nr.robotics.subsystems.intakeRollers;

import edu.nr.lib.commandbased.NRCommand;

public class IntakeRollersStopCommand extends NRCommand {

	/**
	 * Sets motor speed to 0
	 */
	public IntakeRollersStopCommand() {
		
	}
	
	@Override
	protected void onStart() {
		IntakeRollers.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}
	
}
