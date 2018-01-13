package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotor;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotorHDrive;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableOneDDriveMotionProfileTwoMotor extends NRCommand {
	
	Distance initialLeftPosition;
	Distance initialRightPosition;
	Distance initialHPosition;
	Distance tempLeftPosition = Distance.ZERO;
	Distance tempRightPosition = Distance.ZERO;
	
	Distance distanceToDrive = Distance.ZERO;
	
	private final Distance END_THRESHOLD = new Distance(0.3, Distance.Unit.INCH);
	
	public EnableOneDDriveMotionProfileTwoMotor(Distance x) {
		super(Drive.getInstance());
		distanceToDrive = x;
	}
	
	@Override
	public void onStart() {
		Drive.getInstance().enableOneDProfilerTwoMotorH(distanceToDrive);
		initialHPosition = Drive.getInstance().getHPosition();
	}
	
	@Override
	public void onExecute() {
		//System.out.println("1D Profiler enabled: " + Drive.getInstance().isOneDProfilerEnabled());
		Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
		//SmartDashboard.putString("Motion Profiler V Left", Drive.getInstance().pidGetLeft() + ":" + OneDimensionalMotionProfilerTwoMotorHDrive.velocityGoal);
		//SmartDashboard.putString("Motion Profiler V Right", Drive.getInstance().pidGetRight() + ":" + OneDimensionalMotionProfilerTwoMotorHDrive.velocityGoal);
		Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
		//SmartDashboard.putString("Motion Profiler X Left", new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotor.positionGoal + OneDimensionalMotionProfilerTwoMotor.initialPositionLeft, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotor.errorLeft, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));
		//SmartDashboard.putString("Motion Profiler X Right", new Distance(Drive.getInstance().pidGetRight(), Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotor.positionGoal + OneDimensionalMotionProfilerTwoMotor.initialPositionRight, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotor.errorRight, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));	
	}
	
	@Override
	public void onEnd() {
		Drive.getInstance().disableOneDProfilerTwoMotorH();
	}
	
	@Override
	public boolean isFinishedNR() {
		
		boolean finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftPosition())).abs()
				.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getLeftPosition())).abs()
				.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightPosition())).abs()
				.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getRightPosition())).abs()
				.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
				&& Math.abs(((Math.abs(Drive.getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H) - OneDimensionalMotionProfilerTwoMotorHDrive.initialPositionLeft)) - Math.abs(OneDimensionalMotionProfilerTwoMotorHDrive.posPoints.get(OneDimensionalMotionProfilerTwoMotorHDrive.posPoints.size() - 1)))) < END_THRESHOLD.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H);
				
			return finished;
	}


}
