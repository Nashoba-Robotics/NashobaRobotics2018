package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotor;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableOneDMotionProfile extends NRCommand {
	
	Distance initialLeftPosition;
	Distance initialRightPosition;
	Distance tempLeftPosition = Distance.ZERO;
	Distance tempRightPosition = Distance.ZERO;
	
	public EnableOneDMotionProfile() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		Drive.getInstance().enableOneDProfiler(Drive.xProfile);
		initialLeftPosition = Drive.getInstance().getLeftDistance();
		initialRightPosition = Drive.getInstance().getRightDistance();
	}
	
	@Override
	public void onExecute() {
		//System.out.println("1D Profiler enabled: " + Drive.getInstance().isOneDProfilerEnabled());
		Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
		SmartDashboard.putString("Motion Profiler V Left", Drive.getInstance().pidGetLeft() + ":" + OneDimensionalMotionProfilerTwoMotor.velocityGoal);
		SmartDashboard.putString("Motion Profiler V Right", Drive.getInstance().pidGetRight() + ":" + OneDimensionalMotionProfilerTwoMotor.velocityGoal);
		Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
		SmartDashboard.putString("Motion Profiler X Left", Drive.getInstance().pidGetLeft() + ":" + (OneDimensionalMotionProfilerTwoMotor.positionGoal + OneDimensionalMotionProfilerTwoMotor.initialPositionLeft) + ":" + OneDimensionalMotionProfilerTwoMotor.errorLeft);
		SmartDashboard.putString("Motion Profiler X Right", Drive.getInstance().pidGetRight() + ":" + (OneDimensionalMotionProfilerTwoMotor.positionGoal + OneDimensionalMotionProfilerTwoMotor.initialPositionRight) + ":" + OneDimensionalMotionProfilerTwoMotor.errorRight);
	}

	@Override
	public void onEnd() {
		Drive.getInstance().disableOneDProfiler();
	}
	
	@Override
	public boolean isFinishedNR() {
		
		System.out.println((Drive.getInstance().getLeftDistance().sub(new Distance(OneDimensionalMotionProfilerTwoMotor.initialPositionLeft, Distance.Unit.DRIVE_ROTATION).add(Drive.getInstance().getLeftDistance()))).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD));
		
		boolean finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftDistance())).abs()
				.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getLeftDistance())).abs()
				.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightDistance())).abs()
				.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getRightDistance())).abs()
				.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
		//Left off working below
		//		&& (OneDimensionalMotionProfilerTwoMotor.positionGoal.sub(new Distance(OneDimensionalMotionProfilerTwoMotor.initialPositionLeft, Distance.Unit.DRIVE_ROTATION).add(Drive.getInstance().getLeftDistance()))).abs().lessThan(Drive.PROFILE_POSITION_THRESHOLD);
		
		return finished;
	}

}
