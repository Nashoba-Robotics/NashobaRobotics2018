package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveForwardCommandH extends NRCommand {
	
	double driveSpeedPercent;
	Distance startPosition;

	public DriveForwardCommandH() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		
		this.driveSpeedPercent = SmartDashboard.getNumber("Drive Percent H", 0);
		
		Drive.getInstance().setMotorSpeedInPercent(0, 0, driveSpeedPercent);
		
		startPosition = Drive.getInstance().getHDistance();
		
	}
	
	@Override
	public void onExecute() {
		System.out.println(Drive.getInstance().getHVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND));
	}
	
	@Override
	public void onEnd() {
		Drive.getInstance().setMotorSpeed(Speed.ZERO, Speed.ZERO, Speed.ZERO);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return (Drive.getInstance().getHDistance().sub(startPosition)).abs().greaterThan(new Distance(SmartDashboard.getNumber("H Profile Feet", 0), Distance.Unit.FOOT));	
		
	}

}
