package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.TwoDimensionalMotionProfilerPathfinder;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

public class EnableTwoDMotionProfile extends NRCommand {
	
	public EnableTwoDMotionProfile() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		System.out.println("Here");
		Drive.getInstance().enableTwoDProfiler(Drive.xProfile, Drive.yProfile, Drive.endAngle);
	}
	
	@Override
	public void onExecute() {
		//System.out.println("2D Profiler enabled: " + Drive.getInstance().isTwoDProfilerEnabled());
		SmartDashboard.putNumber("Output Left", TwoDimensionalMotionProfilerPathfinder.outputLeft);
		SmartDashboard.putNumber("Output Right", TwoDimensionalMotionProfilerPathfinder.outputRight);
		if(TwoDimensionalMotionProfilerPathfinder.index > 0) {
			SmartDashboard.putString("Motion Profiler Angle", Pathfinder.boundHalfDegrees(TwoDimensionalMotionProfilerPathfinder.currentHeading)+ " : " + Pathfinder.boundHalfDegrees(TwoDimensionalMotionProfilerPathfinder.desiredHeading) + " : " + Pathfinder.boundHalfDegrees(Pathfinder.r2d(TwoDimensionalMotionProfilerPathfinder.modifier.getLeftTrajectory().get(TwoDimensionalMotionProfilerPathfinder.index).heading)));
			SmartDashboard.putString("Motion Profiler X Left String",(Drive.getInstance().pidGetLeft() - TwoDimensionalMotionProfilerPathfinder.initialPositionLeft) + " : " + TwoDimensionalMotionProfilerPathfinder.modifier.getLeftTrajectory().get(TwoDimensionalMotionProfilerPathfinder.index).position);
			SmartDashboard.putString("Motion Profiler X Right String", (Drive.getInstance().pidGetRight() - TwoDimensionalMotionProfilerPathfinder.initialPositionRight) + " : " + TwoDimensionalMotionProfilerPathfinder.modifier.getRightTrajectory().get(TwoDimensionalMotionProfilerPathfinder.index).position);
		}
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
