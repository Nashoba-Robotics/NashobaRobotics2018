package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.units.Distance;

public class DriveForwardSmartDashboardCommandH extends NRCommand {

	Distance initialPosition;
	GyroCorrection gyro;

	public DriveForwardSmartDashboardCommandH() {
		this(Drive.yProfile, Drive.drivePercent);
	}
	
	public DriveForwardSmartDashboardCommandH(Distance distance, double percent) {
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
		double turnValue = gyro.getTurnValue(Drive.kP_thetaOneD);
		Drive.getInstance().setMotorSpeedInPercent(-turnValue, turnValue, Drive.drivePercent);
	}
	
	@Override
	public void onEnd() {
		Drive.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return (Drive.getInstance().getHPosition().sub(initialPosition)).abs().greaterThan(Drive.yProfile);	
		
	}
	
}
