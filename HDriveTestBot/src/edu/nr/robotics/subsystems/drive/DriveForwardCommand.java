package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveForwardCommand extends NRCommand {
	
	double driveSpeedPercent;
	Distance startPosition;

	public DriveForwardCommand() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		
		this.driveSpeedPercent = SmartDashboard.getNumber("Drive Percent", 0);
		
		Drive.getInstance().setMotorSpeedInPercent(driveSpeedPercent, driveSpeedPercent, 0);
		
		startPosition = Drive.getInstance().getRightDistance();
		
	}
	
	@Override
	public void onExecute() {
		System.out.println(Drive.getInstance().getLeftVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND));
	}
	
	@Override
	public void onEnd() {
		Drive.getInstance().setMotorSpeed(Speed.ZERO, Speed.ZERO, Speed.ZERO);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return (Drive.getInstance().getRightDistance().sub(startPosition)).abs().greaterThan(new Distance(SmartDashboard.getNumber("X Profile Feet", 0), Distance.Unit.FOOT));	
		
	}

}
