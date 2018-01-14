package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotor;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotorHDrive;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableOneDMotionProfile extends NRCommand {
	
	Distance initialLeftPosition;
	Distance initialRightPosition;
	Distance tempLeftPosition = Distance.ZERO;
	Distance tempRightPosition = Distance.ZERO;
	
	private final Distance END_THRESHOLD = new Distance(3, Distance.Unit.INCH);
	
	public EnableOneDMotionProfile() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		Drive.getInstance().enableOneDProfiler(Drive.oneDProfileHAssist);
		initialLeftPosition = Drive.getInstance().getLeftDistance();
		initialRightPosition = Drive.getInstance().getRightDistance();
	}
	
	@Override
	public void onExecute() {
		//System.out.println("1D Profiler enabled: " + Drive.getInstance().isOneDProfilerEnabled());
		Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
		SmartDashboard.putString("Motion Profiler V Left", Drive.getInstance().pidGetLeft() + ":" + OneDimensionalMotionProfilerTwoMotorHDrive.velocityGoal);
		SmartDashboard.putString("Motion Profiler V Right", Drive.getInstance().pidGetRight() + ":" + OneDimensionalMotionProfilerTwoMotorHDrive.velocityGoal);
		Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
		SmartDashboard.putString("Motion Profiler X Left", new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotorHDrive.positionGoal + OneDimensionalMotionProfilerTwoMotorHDrive.initialPositionLeft, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotorHDrive.errorLeft, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));
		SmartDashboard.putString("Motion Profiler X Right", new Distance(Drive.getInstance().pidGetRight(), Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotorHDrive.positionGoal + OneDimensionalMotionProfilerTwoMotorHDrive.initialPositionRight, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotorHDrive.errorRight, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));
	}

	@Override
	public void onEnd() {
		Drive.getInstance().disableOneDProfiler();
	}
	
	@Override
	public boolean isFinishedNR() {
				
		boolean finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftDistance())).abs()
			.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
			&& (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getLeftDistance())).abs()
			.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
			&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightDistance())).abs()
			.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
			&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getRightDistance())).abs()
			.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
			&& Math.abs(((Math.abs(Drive.getInstance().getLeftDistance().get(Distance.Unit.MAGNETIC_ENCODER_TICK) - OneDimensionalMotionProfilerTwoMotorHDrive.initialPositionLeft)) - Math.abs(OneDimensionalMotionProfilerTwoMotorHDrive.posPoints.get(OneDimensionalMotionProfilerTwoMotorHDrive.posPoints.size() - 1)))) < END_THRESHOLD.get(Distance.Unit.MAGNETIC_ENCODER_TICK);
		return finished;
	}

}
