package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.HDriveDiagonalProfiler;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerHDriveMain;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotorHDrive;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableMotionProfile extends NRCommand {
	
	Distance initialLeftPosition;
	Distance initialRightPosition;
	Distance initialHPosition;
	Distance tempLeftPosition = Distance.ZERO;
	Distance tempRightPosition = Distance.ZERO;
	Distance tempHPosition = Distance.ZERO;
	
	private final Distance END_THRESHOLD = new Distance(3, Distance.Unit.INCH);
	
	public EnableMotionProfile() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		Drive.getInstance().enableProfiler(Drive.profileDistanceX, Drive.profileDistanceY);
		initialLeftPosition = Drive.getInstance().getLeftDistance();
		initialRightPosition = Drive.getInstance().getRightDistance();
		initialHPosition = Drive.getInstance().getHDistance();
		//System.out.println("profiler starting");
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
		
			finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftDistance())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getLeftDistance())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightDistance())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getRightDistance())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& Math.abs((((Math.abs(Drive.getInstance().getLeftDistance().get(Distance.Unit.MAGNETIC_ENCODER_TICK) - initialLeftPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK) + Drive.getInstance().getRightDistance().get(Distance.Unit.MAGNETIC_ENCODER_TICK) - initialRightPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK)))/2) - Math.abs(HDriveDiagonalProfiler.posPoints.get(HDriveDiagonalProfiler.posPoints.size() - 1)))) < END_THRESHOLD.get(Distance.Unit.MAGNETIC_ENCODER_TICK)
					&& (Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getHDistance())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getHDistance())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& Math.abs(((Math.abs(Drive.getInstance().getHDistance().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H) - initialHPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H))) - Math.abs(HDriveDiagonalProfiler.posPointsH.get(HDriveDiagonalProfiler.posPointsH.size() - 1)))) < END_THRESHOLD.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H);
		return finished;
	}

}
