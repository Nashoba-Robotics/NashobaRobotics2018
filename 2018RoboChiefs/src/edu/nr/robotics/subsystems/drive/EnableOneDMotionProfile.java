package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.HDriveDiagonalProfiler;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerHDriveMain;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotorHDrive;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableOneDMotionProfile extends NRCommand {
	
	Distance initialLeftPosition;
	Distance initialRightPosition;
	Distance initialHPosition;
	Distance tempLeftPosition = Distance.ZERO;
	Distance tempRightPosition = Distance.ZERO;
	Distance tempHPosition = Distance.ZERO;
	
	Distance xDistance;
	Distance yDistance;
	double maxVelPercent;
	double maxAccelPercent;
	
	private final Distance END_THRESHOLD = new Distance(3, Distance.Unit.INCH);
	
	public EnableOneDMotionProfile(Distance xDistance, Distance yDistance, double maxVelPercent, double maxAccelPercent) {
		super(Drive.getInstance());
		this.xDistance = xDistance;
		this.yDistance = yDistance;
		this.maxVelPercent = maxVelPercent;
		this.maxAccelPercent = maxAccelPercent;
	}
	
	@Override
	public void onStart() {
		Drive.getInstance().enableMotionProfiler(xDistance, yDistance, maxVelPercent, maxAccelPercent);
		initialLeftPosition = Drive.getInstance().getLeftPosition();
		initialRightPosition = Drive.getInstance().getRightPosition();
		initialHPosition = Drive.getInstance().getHPosition();
	}

	@Override
	public void onExecute() {
		// System.out.println("1D Profiler enabled: " +
		// Drive.getInstance().isOneDProfilerEnabled());
		Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
		SmartDashboard.putString("Motion Profiler V Left", Drive.getInstance().pidGetLeft() + ":" + HDriveDiagonalProfiler.velocityGoal);
		SmartDashboard.putString("Motion Profiler V Right", Drive.getInstance().pidGetRight() + ":" + HDriveDiagonalProfiler.velocityGoal);
		SmartDashboard.putString("Motion Profiler V H", Drive.getInstance().pidGetH() + ":" + HDriveDiagonalProfiler.velocityGoalH);
		Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
		SmartDashboard.putString("Motion Profiler X Left", new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(HDriveDiagonalProfiler.positionGoal + HDriveDiagonalProfiler.initialPositionLeft, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(HDriveDiagonalProfiler.errorLeft, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));
		SmartDashboard.putString("Motion Profiler X Right", new Distance(Drive.getInstance().pidGetRight(), Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(HDriveDiagonalProfiler.positionGoal + HDriveDiagonalProfiler.initialPositionRight, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(HDriveDiagonalProfiler.errorRight, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));
		SmartDashboard.putString("Motion Profiler X H", new Distance(Drive.getInstance().pidGetH(), Distance.Unit.MAGNETIC_ENCODER_TICK_H).get(Distance.Unit.INCH) + ":" + new Distance(HDriveDiagonalProfiler.positionGoalH + HDriveDiagonalProfiler.initialPositionH, Distance.Unit.MAGNETIC_ENCODER_TICK_H).get(Distance.Unit.INCH) + ":" + new Distance(HDriveDiagonalProfiler.errorH, Distance.Unit.MAGNETIC_ENCODER_TICK_H).get(Distance.Unit.INCH));
	}

	@Override
	public void onEnd() {
		Drive.getInstance().disableProfiler();
	}

	@Override
	public boolean isFinishedNR() {
		boolean finished = false;
		
		finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftPosition())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getLeftPosition())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightPosition())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getRightPosition())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& Math.abs((((Math.abs(Drive.getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK) - initialLeftPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK) + Drive.getInstance().getRightPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK) - initialRightPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK)))/2) - Math.abs(HDriveDiagonalProfiler.posPoints.get(HDriveDiagonalProfiler.posPoints.size() - 1)))) < END_THRESHOLD.get(Distance.Unit.MAGNETIC_ENCODER_TICK)
				&& (Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getHPosition())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& (Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getHPosition())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& Math.abs(((Math.abs(Drive.getInstance().getHPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H) - initialHPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H))) - Math.abs(HDriveDiagonalProfiler.posPointsH.get(HDriveDiagonalProfiler.posPointsH.size() - 1)))) < END_THRESHOLD.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H);
	return finished;
	}

}
