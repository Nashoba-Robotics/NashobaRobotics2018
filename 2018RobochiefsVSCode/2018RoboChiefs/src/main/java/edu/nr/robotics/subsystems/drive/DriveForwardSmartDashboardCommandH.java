package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.units.Distance;

public class DriveForwardSmartDashboardCommandH extends NRCommand {

	Distance initialPosition;
	GyroCorrection gyro;

	public DriveForwardSmartDashboardCommandH() {
		super(Drive.getInstance());
		gyro = new GyroCorrection();
	}
	
	@Override
	public void onStart() {
		initialPosition = Drive.getInstance().getHPosition();
		gyro.reset();
	}
	
	@Override
	public void onExecute() {
		double turnValue = gyro.getTurnValue(Drive.kP_thetaOneD, false);
		Drive.getInstance().setMotorSpeedInPercent(-turnValue, turnValue, Drive.drivePercent * Drive.yProfile.signum());
	}
	
	@Override
	public void onEnd() {
		Drive.getInstance().setMotorSpeedInPercent(0, 0, 0);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return (Drive.getInstance().getHPosition().sub(initialPosition)).abs().greaterThan(Drive.yProfile.abs());	
		
	}
	
}
