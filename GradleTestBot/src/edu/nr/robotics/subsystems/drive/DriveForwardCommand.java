package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
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
		
		Drive.getInstance().setMotorSpeedInPercent(driveSpeedPercent, driveSpeedPercent);
		
		startPosition = Drive.getInstance().getRightDistance();
		
	}
	
	@Override
	protected boolean isFinishedNR() {
		System.out.println(Drive.getInstance().getRightDistance().sub(startPosition));
		return (Drive.getInstance().getRightDistance().sub(startPosition)).greaterThan(new Distance(SmartDashboard.getNumber("Distance to Profile in Feet", 0), Distance.Unit.FOOT));
			
		
	}

}
