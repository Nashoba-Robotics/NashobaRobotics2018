package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class EnableTwoDMotionProfile extends NRCommand {
	
	public EnableTwoDMotionProfile() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		Drive.getInstance().enableTwoDProfiler(Drive.xProfile, Drive.yProfile, Drive.endAngle);
	}
	
	@Override
	public void onExecute() {
		//System.out.println("2D Profiler enabled: " + Drive.getInstance().isTwoDProfilerEnabled());
	}

	@Override
	public void onEnd() {
		Drive.getInstance().disableTwoDProfiler();
	}
	
	@Override
	public boolean isFinishedNR() {
		return false;
	}

}
