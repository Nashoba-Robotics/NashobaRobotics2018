package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.drive.Drive;

public class IntakeWhileSlowingCommand extends NRCommand {

	private boolean slowedEnough = false;
	private Speed prevSpeed;
	private Speed lowEnoughSpeedThreshold = new Speed(6, Distance.Unit.FOOT, Time.Unit.SECOND);
	
	public IntakeWhileSlowingCommand() {
		super(IntakeElevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		prevSpeed = Drive.getInstance().getLeftVelocity();
		slowedEnough = false;
	}
	
	@Override
	protected void onExecute() {
		
		if (!slowedEnough && prevSpeed.greaterThan(Drive.getInstance().getLeftVelocity()) && Drive.getInstance().getLeftVelocity().lessThan(lowEnoughSpeedThreshold)) {
			slowedEnough = true;
		}
		
		if (slowedEnough) {
			IntakeElevator.getInstance().setMotorSpeedPercent(-0.3);
			IntakeElevator.intakeFolded = false;
		}
		
		prevSpeed = Drive.getInstance().getLeftVelocity();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return IntakeElevator.getInstance().getPosition().equals(Distance.ZERO);
	}
}
