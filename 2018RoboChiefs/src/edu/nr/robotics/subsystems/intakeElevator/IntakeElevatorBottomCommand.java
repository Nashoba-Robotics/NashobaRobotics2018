package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.Timer;

public class IntakeElevatorBottomCommand extends NRCommand {
	
	private double initHitBottomTime = 0;
	private boolean atBottom = false;
	private boolean hitBottom = false;

	public IntakeElevatorBottomCommand() {
		super(IntakeElevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		IntakeElevator.getInstance().setMotorSpeedPercent(-IntakeElevator.DRIVE_TO_BOTTOM_PERCENT_INTAKE_ELEVATOR);
		IntakeElevator.intakeFolded = false;
		initHitBottomTime = 0;
		atBottom = false;
		hitBottom = false;
	}
	
	@Override
	protected void onExecute() {
		if (IntakeElevator.getInstance().getCurrent() >= IntakeElevator.HIT_BOTTOM_CURRENT_INTAKE_ELEVATOR) {
			if (!hitBottom) {
				hitBottom = true;
				initHitBottomTime = Timer.getFPGATimestamp();
			} else {
				if ((Timer.getFPGATimestamp() - initHitBottomTime) * 1000 >= IntakeElevator.CONTINUOUS_CURRENT_LIMIT_INTAKE_ELEVATOR) {
					atBottom = true;
				}
			}
		
		} else {
			hitBottom = false;
		}
	}
	
	@Override
	protected boolean isFinishedNR() {
		return IntakeElevator.getInstance().getPosition().equals(Distance.ZERO) || atBottom;
	}
}
