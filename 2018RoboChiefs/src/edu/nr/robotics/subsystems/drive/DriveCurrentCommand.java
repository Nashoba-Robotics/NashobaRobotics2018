package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;

public class DriveCurrentCommand extends NRCommand {

	private double currentLimit;
	private double drivePercent;
	
	public DriveCurrentCommand(double drivePercent, double currentLimit) {
		super(Drive.getInstance());
		this.currentLimit = currentLimit;
		this.drivePercent = drivePercent;
	}
	
	@Override
	protected void onStart() {
		Drive.getInstance().setMotorSpeedInPercent(drivePercent, drivePercent, 0);
	}
	
	@Override
	protected void onEnd() {
		Drive.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return Drive.getInstance().getLeftCurrent() > currentLimit || Drive.getInstance().getRightCurrent() > currentLimit;
	}
}
