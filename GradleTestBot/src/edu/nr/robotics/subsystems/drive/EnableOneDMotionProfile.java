package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableOneDMotionProfile extends NRCommand {
	
	public static final Distance PROFILE_POSITION_THRESHOLD = new Distance(1, Distance.Unit.INCH);
	
	Distance initialLeftPosition;
	Distance initialRightPosition;
	Distance tempLeftPosition = Distance.ZERO;
	Distance tempRightPosition = Distance.ZERO;
	
	public EnableOneDMotionProfile() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		Drive.getInstance().enableOneDProfiler(new Distance(SmartDashboard.getNumber("Distance to Profile in Feet", 0), Distance.Unit.FOOT));
		initialLeftPosition = Drive.getInstance().getLeftDistance();
		initialRightPosition = Drive.getInstance().getRightDistance();
	}
	
	@Override
	public void onExecute() {
		//System.out.println("1D Profiler enabled: " + Drive.getInstance().isOneDProfilerEnabled());
	}

	@Override
	public void onEnd() {
		Drive.getInstance().disableOneDProfiler();
	}
	
	@Override
	public boolean isFinishedNR() {
		/*boolean finished = Drive.getInstance().getLeftDistance().sub(initialLeftPosition).abs().greaterThan(PROFILE_POSITION_THRESHOLD)
				&& Drive.getInstance().getRightDistance().sub(initialRightPosition).abs().greaterThan(PROFILE_POSITION_THRESHOLD)
				&& Drive.getInstance().getLeftDistance().equals(tempLeftPosition)
				&& Drive.getInstance().getRightDistance().equals(tempRightPosition);
		tempLeftPosition = Drive.getInstance().getLeftDistance();
		tempRightPosition = Drive.getInstance().getRightDistance();
		*/
		return false;
	}

}
