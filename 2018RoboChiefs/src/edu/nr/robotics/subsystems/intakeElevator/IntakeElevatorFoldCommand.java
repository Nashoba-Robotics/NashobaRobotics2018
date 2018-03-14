package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;

public class IntakeElevatorFoldCommand extends NRCommand {
	
	public IntakeElevatorFoldCommand() {
		super(IntakeElevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		IntakeElevator.getInstance().setMotorSpeedPercent(0.6);
		IntakeElevator.intakeFolded = true;
	}
	
	@Override
	protected boolean isFinishedNR() {
		return IntakeElevator.getInstance().getCurrent() >= IntakeElevator.FOLD_CURRENT_SPIKE;
	}

}