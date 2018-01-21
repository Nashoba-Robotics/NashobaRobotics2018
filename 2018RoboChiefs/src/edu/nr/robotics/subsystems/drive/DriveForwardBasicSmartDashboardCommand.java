package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.units.Distance;

public class DriveForwardBasicSmartDashboardCommand extends NRCommand {
	
	Distance initialPosition;
	GyroCorrection gyro;
	
	public DriveForwardBasicSmartDashboardCommand() {
		this(Drive.xProfile, Drive.drivePercent);
	}
	
	public DriveForwardBasicSmartDashboardCommand(Distance distance, double percent) {
		super(Drive.getInstance());
		gyro = new GyroCorrection();
	}
	
	
	
	@Override
	public void onStart() {
		initialPosition = Drive.getInstance().getLeftPosition();
		gyro.reset();

	}
	
	@Override
	public void onExecute() {
		double turnValue = gyro.getTurnValue(Drive.kP_thetaOneD);
		Drive.getInstance().setMotorSpeedInPercent(Drive.drivePercent - turnValue, Drive.drivePercent + turnValue, 0);
	}
	
	public void onEnd() {
		Drive.getInstance().disable();
	}
	
	@Override
	public boolean isFinishedNR() {
		return (Drive.getInstance().getLeftPosition().sub(initialPosition)).abs().greaterThan(Drive.xProfile);
	}

}
