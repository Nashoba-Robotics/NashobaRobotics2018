package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveForwardCommand extends NRCommand {
	
	double driveSpeedPercentLeft;
	double driveSpeedPercentRight;
	Distance startPosition;

	public DriveForwardCommand() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		
		this.driveSpeedPercentLeft = SmartDashboard.getNumber("Drive Percent Left", 0);
		this.driveSpeedPercentRight = SmartDashboard.getNumber("Drive Percent Right", 0);
		
		Drive.getInstance().setMotorSpeedInPercent(driveSpeedPercentLeft, driveSpeedPercentRight);
		
		startPosition = Drive.getInstance().getRightDistance();
		
	}
	
	@Override
	protected boolean isFinishedNR() {
		System.out.println(Drive.getInstance().getRightDistance().sub(startPosition));
		return (Drive.getInstance().getRightDistance().sub(startPosition)).greaterThan(new Distance(20, Distance.Unit.FOOT));
			
		
	}

}
