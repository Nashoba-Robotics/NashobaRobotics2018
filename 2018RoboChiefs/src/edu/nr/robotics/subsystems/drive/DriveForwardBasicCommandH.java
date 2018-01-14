package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveForwardBasicCommandH extends NRCommand {
	
	double driveSpeedPercent;
	Distance distance;
	Distance initialPosition;
	GyroCorrection gyro;

	public DriveForwardBasicCommandH(Distance distance) {
		this(distance, Drive.PROFILE_DRIVE_PERCENT);
	}
	
	public DriveForwardBasicCommandH(Distance distance, double percent) {
		super(Drive.getInstance());
		this.driveSpeedPercent = percent;
		this.distance = distance;
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
		Drive.getInstance().setMotorSpeedInPercent(-turnValue, turnValue, driveSpeedPercent);
	}
	
	@Override
	public void onEnd() {
		Drive.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return (Drive.getInstance().getHPosition().sub(initialPosition)).abs().greaterThan(distance);	
		
	}

}
