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
	
	private final Distance END_THRESHOLD = new Distance(3, Distance.Unit.INCH);
	
	public EnableOneDMotionProfile(Distance dist, Distance distH, maxVelPercent, maxAccelPercent) {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		Drive.getInstance().enableMotionProfiler(Drive.xProfile, Drive.yProfile);
		initialLeftPosition = Drive.getInstance().getLeftPosition();
		initialRightPosition = Drive.getInstance().getRightPosition();
		initialHPosition = Drive.getInstance().getHPosition();
	}

	@Override
	public void onExecute() {
		// System.out.println("1D Profiler enabled: " +
		// Drive.getInstance().isOneDProfilerEnabled());
		Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
		SmartDashboard.putString("Motion Profiler V Left", Drive.getInstance().pidGetLeft() + ":" + OneDimensionalMotionProfilerTwoMotorHDrive.velocityGoal);
		SmartDashboard.putString("Motion Profiler V Right", Drive.getInstance().pidGetRight() + ":" + OneDimensionalMotionProfilerTwoMotorHDrive.velocityGoal);
		Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
		SmartDashboard.putString("Motion Profiler X Left", new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotorHDrive.positionGoal + OneDimensionalMotionProfilerTwoMotorHDrive.initialPositionLeft, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotorHDrive.errorLeft, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));
		SmartDashboard.putString("Motion Profiler X Right", new Distance(Drive.getInstance().pidGetRight(), Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotorHDrive.positionGoal + OneDimensionalMotionProfilerTwoMotorHDrive.initialPositionRight, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotorHDrive.errorRight, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));
	}

	@Override
	public void onEnd() {
		Drive.getInstance().disableProfiler();
	}

	@Override
	public boolean isFinishedNR() {
		boolean finished = false;

		if (Drive.xProfile != Distance.ZERO && Drive.yProfile == Distance.ZERO) {
			finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD)
					.sub(Drive.getInstance().getLeftPosition())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2))
							.sub(Drive.getInstance().getLeftPosition())).abs()
									.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD)
							.sub(Drive.getInstance().getRightPosition())).abs()
									.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2))
							.sub(Drive.getInstance().getRightPosition())).abs()
									.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& Math.abs(((Math
							.abs(Drive.getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK)
									- OneDimensionalMotionProfilerTwoMotorHDrive.initialPositionLeft))
							- Math.abs(OneDimensionalMotionProfilerTwoMotorHDrive.posPoints.get(
									OneDimensionalMotionProfilerTwoMotorHDrive.posPoints.size() - 1)))) < END_THRESHOLD
											.get(Distance.Unit.MAGNETIC_ENCODER_TICK);
		} else if (Drive.xProfile == Distance.ZERO && Drive.yProfile != Distance.ZERO) {
			finished = (Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD)
					.sub(Drive.getInstance().getHPosition())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2))
							.sub(Drive.getInstance().getHPosition())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& Math.abs(((Math.abs(Drive.getInstance().getHPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H)
							- OneDimensionalMotionProfilerHDriveMain.initialPositionH))
							- Math.abs(OneDimensionalMotionProfilerHDriveMain.posPoints
									.get(OneDimensionalMotionProfilerHDriveMain.posPoints.size() - 1)))) < END_THRESHOLD
											.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H);
		} else if (Drive.xProfile != Distance.ZERO && Drive.yProfile != Distance.ZERO) {
			finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD)
					.sub(Drive.getInstance().getLeftPosition())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2))
							.sub(Drive.getInstance().getLeftPosition())).abs()
									.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD)
							.sub(Drive.getInstance().getRightPosition())).abs()
									.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2))
							.sub(Drive.getInstance().getRightPosition())).abs()
									.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& Math.abs(((Math
							.abs(Drive.getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK)
									- HDriveDiagonalProfiler.initialPositionLeft))
							- Math.abs(HDriveDiagonalProfiler.posPoints.get(
									HDriveDiagonalProfiler.posPoints.size() - 1)))) < END_THRESHOLD
											.get(Distance.Unit.MAGNETIC_ENCODER_TICK)
					&& (Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD)
							.sub(Drive.getInstance().getHPosition())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2))
							.sub(Drive.getInstance().getHPosition())).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& Math.abs(((Math.abs(Drive.getInstance().getHPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H)
							- HDriveDiagonalProfiler.initialPositionH))
							- Math.abs(HDriveDiagonalProfiler.posPoints
									.get(HDriveDiagonalProfiler.posPoints.size() - 1)))) < END_THRESHOLD
											.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H);
		} else {
			finished = true;
			System.out.println("No distances set");
		}
		return finished;
	}

}
