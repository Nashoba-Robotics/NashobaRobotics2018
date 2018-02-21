package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;

public class IntakeElevatorFoldCommand extends NRCommand {
	
	private boolean folding = false;
	private boolean foldFinished = false;
	
	public IntakeElevatorFoldCommand() {
		super(IntakeElevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		IntakeElevator.getInstance().setMotorSpeedPercent(0.6);
	}
	
	@Override
	protected void onExecute() {
		if (IntakeElevator.getInstance().getCurrent() >= IntakeElevator.FOLD_CURRENT_SPIKE) {
			folding = true;
		}
		
		if((IntakeElevator.getInstance().getCurrent() <= IntakeElevator.FOLD_FINISHED_CURRENT) && folding) {
			foldFinished = true;
		}
		
	}
	
	@Override
	protected boolean isFinishedNR() {
		return ((IntakeElevator.getInstance().getCurrent() >= IntakeElevator.HIT_TOP_HEIGHT_CURRENT) && foldFinished);
	}

}
