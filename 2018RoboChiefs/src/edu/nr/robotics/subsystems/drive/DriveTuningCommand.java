package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTuningCommand extends NRCommand {

	private double counter = -1;
	
	private double minPercent = 0.20;
	private double percentInterval = 0.20;
	
	private static final Distance SLOW_DIST = new Distance(10, Distance.Unit.FOOT);
	private static final Distance FAST_DIST = new Distance(20, Distance.Unit.FOOT);
	private static final double DIST_CHANGE_THRESHOLD = 0.50;
	
	public DriveTuningCommand() {
		counter = -1;
	}
	
	@Override
	protected void onStart() {
		counter ++;
		if (counter * percentInterval + minPercent < DIST_CHANGE_THRESHOLD) {
			new DriveForwardBasicCommand(SLOW_DIST, counter * percentInterval + minPercent).start();
		} else {
			new DriveForwardBasicCommand(FAST_DIST, counter * percentInterval + minPercent).start();
		}
	}
	
	@Override
	protected void onExecute() {
		if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Left Drive Tuning Speed", Drive.getInstance().getLeftVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND));
			SmartDashboard.putNumber("Right Drive Tuning Speed", Drive.getInstance().getRightVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND));
		}
	}
	
	@Override
	protected boolean isFinishedNR() {
		return !(Drive.getInstance().getCurrentCommand() instanceof DriveForwardBasicCommand);
	}
	
}
