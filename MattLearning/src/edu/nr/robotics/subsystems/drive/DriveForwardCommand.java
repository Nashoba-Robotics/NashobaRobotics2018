package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveForwardCommand extends NRCommand{

	double driveSpeedPercent;
	Distance startPosition;
	
	public DriveForwardCommand() {
		super(Drive.getInstance());
	}
	
	public void onStart() {
		
		this.driveSpeedPercent = SmartDashboard.getNumber("Drive Percent", 0);
		
		Drive.getInstance().setMotorSpeedInPercent(driveSpeedPercent, driveSpeedPercent);
		
		startPosition = Drive.getInstance().getRightPosition(); //may be wrong because nick changed it and he wasn't sure
		
	}
	
	public void onExecute() {
		System.out.println(Drive.getInstance().getLeftVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND));
	}
	
	public void onEnd() {
		Drive.getInstance().setMotorSpeed(Speed.ZERO, Speed.ZERO);
	}
	
	protected boolean isFinishedNR() {
		return (Drive.getInstance().getRightPosition().sub(startPosition)).abs().greaterThan(new Distance(SmartDashboard.getNumber("X Profile Feet", 0), Distance.Unit.FOOT));	
		//may be wrong nick changed getRightDistance(), something that doesn't exist, to getRightPosition()
	}
	
	
}
