package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.TwoDimensionalMotionProfilerPathfinder;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableTwoDMotionProfileSmartDashboardCommand extends NRCommand {
	
	Distance initialLeftPosition;
	Distance initialRightPosition;
	Distance tempLeftPosition = Distance.ZERO;
	Distance tempRightPosition = Distance.ZERO;
	
	private final Distance END_THRESHOLD = Drive.END_THRESHOLD;
	
	public EnableTwoDMotionProfileSmartDashboardCommand() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		System.out.println("starts");
		Drive.getInstance().enableMotionProfiler(Drive.xProfile, Drive.yProfile, Drive.endAngle, Drive.drivePercent, Drive.accelPercent);
		initialLeftPosition = Drive.getInstance().getLeftPosition();
		initialRightPosition = Drive.getInstance().getRightPosition();
		System.out.println("end starts");
	}

	@Override
	public void onExecute() {
		System.out.println("starts execute");
			Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
			SmartDashboard.putString("Motion Profiler V Left",
					Drive.getInstance().pidGetLeft() + ":" + TwoDimensionalMotionProfilerPathfinder.velocityGoalLeft);
			SmartDashboard.putString("Motion Profiler V Right",
					Drive.getInstance().pidGetRight() + ":" + TwoDimensionalMotionProfilerPathfinder.velocityGoalRight);
			Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
			/*SmartDashboard.putString("Motion Profiler X Left",
					new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
							.get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(
									TwoDimensionalMotionProfilerPathfinder.positionGoalLeft + TwoDimensionalMotionProfilerPathfinder.initialPositionLeft,
									Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH)
							+ ":" + new Distance(TwoDimensionalMotionProfilerPathfinder.errorLeft, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
									.get(Distance.Unit.INCH));
			SmartDashboard.putString("Motion Profiler X Right",
					new Distance(Drive.getInstance().pidGetRight(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
							.get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(
									TwoDimensionalMotionProfilerPathfinder.positionGoalRight + TwoDimensionalMotionProfilerPathfinder.initialPositionRight,
									Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH)
							+ ":" + new Distance(TwoDimensionalMotionProfilerPathfinder.errorRight, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
									.get(Distance.Unit.INCH));*/

		
	}
	@Override
	public void onEnd() {
		System.out.println("ends");
		Drive.getInstance().disableProfiler();
		Drive.getInstance().setMotorSpeedInPercent(0, 0);
	}

	@Override
	public boolean isFinishedNR() {

		boolean finished;

		finished = Math
				.abs(Math.abs(Drive.getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
						- initialLeftPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE))) > (NRMath
								.hypot(Drive.xProfile, Drive.yProfile).sub(Drive.END_THRESHOLD))
										.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
				|| Math.abs(
						Math.abs(Drive.getInstance().getRightPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
								- initialLeftPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE))) > (NRMath
										.hypot(Drive.xProfile, Drive.yProfile).sub(Drive.END_THRESHOLD)
										.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE))

						&& Drive.getInstance().getLeftVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD)
						&& Drive.getInstance().getRightVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD);
		
		return finished;
	}

}
