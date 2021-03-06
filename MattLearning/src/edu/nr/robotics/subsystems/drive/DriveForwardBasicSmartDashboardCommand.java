package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;

public class DriveForwardBasicSmartDashboardCommand extends NRCommand {

	Distance initialPosition;
	GyroCorrection gyro;
	
	public DriveForwardBasicSmartDashboardCommand() {
		super(Drive.getInstance());
		gyro = new GyroCorrection();
	}
	
	public void onStart() {
		initialPosition = Drive.getInstance().getLeftPosition();
		gyro.reset();

	}
	
	public void onExecute() {
		double turnValue = gyro.getTurnValue(Drive.kP_thetaOneD, false);
		Drive.getInstance().setMotorSpeedInPercent(Drive.drivePercent * Drive.xProfile.signum() - turnValue, 
				Drive.drivePercent * Drive.xProfile.signum() + turnValue);
		System.out.println(Drive.getInstance().getRightVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND));
		
		
	}
	
	public void onEnd() {
		Drive.getInstance().setMotorSpeedInPercent(0,0);
	}
	
	public boolean isFinishedNR() {
		return (Drive.getInstance().getLeftPosition().sub(initialPosition)).abs().greaterThan(Drive.xProfile.abs());
	}
	
}
