package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.wpi.first.wpilibj.Timer;

public class IntakeElevatorFoldCommand extends NRCommand {
	
	private double initFoldTime = 0;
	private boolean folding = false;
	private boolean isFolded = false;
	
	public IntakeElevatorFoldCommand() {
		super(IntakeElevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		IntakeElevator.getInstance().setMotorSpeedPercent(0.4);
		IntakeElevator.intakeFolded = true;
		initFoldTime = 0;
		folding = false;
		isFolded = false;
	}
	
	@Override
	protected void onExecute() {
		if (IntakeElevator.getInstance().getCurrent() >= IntakeElevator.FOLD_CURRENT_SPIKE) {
			if (!folding) {
				folding = true;
				initFoldTime = Timer.getFPGATimestamp();
				
			} else {
				if ((Timer.getFPGATimestamp() - initFoldTime) * 1000 >= IntakeElevator.PEAK_CURRENT_DURATION_INTAKE_ELEVATOR) {
					isFolded = true;
				}
			}
		}
	}
	
	@Override
	protected boolean isFinishedNR() {
		return isFolded;
	}

}
