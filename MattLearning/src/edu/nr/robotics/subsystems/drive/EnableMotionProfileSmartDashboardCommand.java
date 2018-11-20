package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotor;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableMotionProfileSmartDashboardCommand extends NRCommand {
	
	Distance initialLeftPosition;
	Distance initialRightPosition;
	Distance tempLeftPosition = Distance.ZERO;
	Distance tempRightPosition = Distance.ZERO;
	
	public EnableMotionProfileSmartDashboardCommand() {
		super(Drive.getInstance());
	}
	
	public void onStart() {
		Drive.getInstance().enableMotionProfiler(Drive.xProfile, Drive.yProfile, Drive.drivePercent, Drive.accelPercent);
		initialLeftPosition = Drive.getInstance().getLeftPosition();
		initialRightPosition = Drive.getInstance().getRightPosition();
	}
	
	public void onExecute() {
		if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
			Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
			SmartDashboard.putString("Motion Profiler V Left",
					Drive.getInstance().pidGetLeft() + ":" + OneDimensionalMotionProfilerTwoMotor.velocityGoal);
			SmartDashboard.putString("Motion Profiler V Right",
					Drive.getInstance().pidGetRight() + ":" + OneDimensionalMotionProfilerTwoMotor.velocityGoal);
			Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
			SmartDashboard.putString("Motion Profiler X Left",
					new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
							.get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(
									OneDimensionalMotionProfilerTwoMotor.positionGoal + OneDimensionalMotionProfilerTwoMotor.initialPositionLeft,
									Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH)
							+ ":" + new Distance(OneDimensionalMotionProfilerTwoMotor.errorLeft, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
									.get(Distance.Unit.INCH));
			SmartDashboard.putString("Motion Profiler X Right",
					new Distance(Drive.getInstance().pidGetRight(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
							.get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(
									OneDimensionalMotionProfilerTwoMotor.positionGoal + OneDimensionalMotionProfilerTwoMotor.initialPositionRight,
									Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH)
							+ ":" + new Distance(OneDimensionalMotionProfilerTwoMotor.errorRight, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
									.get(Distance.Unit.INCH));

		}
	}
	
	public void onEnd() {
		Drive.getInstance().disableProfiler();
		Drive.getInstance().setMotorSpeedInPercent(0, 0);
	}
	
	public boolean isFinishedNR() {
		
		boolean finished;
		
		finished = Math.abs(Drive.getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
				- initialLeftPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE) - OneDimensionalMotionProfilerTwoMotor.posPoints
						.get(OneDimensionalMotionProfilerTwoMotor.posPoints.size() - 1)) < Drive.END_THRESHOLD
				.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE) && 
				Math.abs(Drive.getInstance().getRightPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
				- initialRightPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE) - OneDimensionalMotionProfilerTwoMotor.posPoints
						.get(OneDimensionalMotionProfilerTwoMotor.posPoints.size() - 1)) < Drive.END_THRESHOLD
				.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
								&& Drive.getInstance().getLeftVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD)
				&& Drive.getInstance().getRightVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD);
		
		return finished;
	}
	
	
	
	
	
}
