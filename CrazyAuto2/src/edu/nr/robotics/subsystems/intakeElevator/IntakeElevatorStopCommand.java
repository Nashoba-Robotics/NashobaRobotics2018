package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;

public class IntakeElevatorStopCommand extends NRCommand {
	
	/**
	 * Sets the elevator speed to 0
	 */
	public IntakeElevatorStopCommand() {
		super(IntakeElevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		IntakeElevator.getInstance().setMotorSpeedPercent(0);
	}
	
	@Override 
	protected boolean isFinishedNR() {
		return true;
	}

}
