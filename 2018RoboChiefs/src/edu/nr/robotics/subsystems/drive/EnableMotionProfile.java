package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.HDriveDiagonalProfiler;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableMotionProfile extends NRCommand {
	
	Distance initialLeftPosition;
	Distance initialRightPosition;
	Distance initialHPosition;
	Distance tempLeftPosition = Distance.ZERO;
	Distance tempRightPosition = Distance.ZERO;
	Distance tempHPosition = Distance.ZERO;
	
	Distance dist;
	Distance distH;
	double maxVelPercent;
	double maxAccelPercent;
		
	public EnableMotionProfile(Distance dist, Distance distH, double maxVelPercent, double maxAccelPercent) {
		super(Drive.getInstance());
		this.dist = dist;
		this.distH = distH;
		this.maxVelPercent = maxVelPercent;
		this.maxAccelPercent = maxAccelPercent;
	}
	
	@Override
	public void onStart() {
		Drive.getInstance().enableMotionProfiler(dist, distH, maxVelPercent, maxAccelPercent);
		initialLeftPosition = Drive.getInstance().getLeftPosition();
		initialRightPosition = Drive.getInstance().getRightPosition();
		initialHPosition = Drive.getInstance().getHPosition();
	}

	@Override
	public void onExecute() {
		if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {	
			Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
			SmartDashboard.putString("Motion Profiler V Left",
					Drive.getInstance().pidGetLeft() + ":" + HDriveDiagonalProfiler.velocityGoal);
			SmartDashboard.putString("Motion Profiler V Right",
					Drive.getInstance().pidGetRight() + ":" + HDriveDiagonalProfiler.velocityGoal);
			SmartDashboard.putString("Motion Profiler V H",
					Drive.getInstance().pidGetH() + ":" + HDriveDiagonalProfiler.velocityGoalH);
			Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
			SmartDashboard.putString("Motion Profiler X Left",
					new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
							.get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(
									HDriveDiagonalProfiler.positionGoal + HDriveDiagonalProfiler.initialPositionLeft,
									Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(HDriveDiagonalProfiler.errorLeft, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
									.get(Distance.Unit.INCH));
			SmartDashboard.putString("Motion Profiler X Right",
					new Distance(Drive.getInstance().pidGetRight(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
							.get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(
									HDriveDiagonalProfiler.positionGoal + HDriveDiagonalProfiler.initialPositionRight,
									Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(HDriveDiagonalProfiler.errorRight, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
									.get(Distance.Unit.INCH));
			SmartDashboard.putString("Motion Profiler X H",
					new Distance(Drive.getInstance().pidGetH(), Distance.Unit.MAGNETIC_ENCODER_TICK_H)
							.get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(
									HDriveDiagonalProfiler.positionGoalH + HDriveDiagonalProfiler.initialPositionH,
									Distance.Unit.MAGNETIC_ENCODER_TICK_H).get(Distance.Unit.INCH)
							+ ":" + new Distance(HDriveDiagonalProfiler.errorH, Distance.Unit.MAGNETIC_ENCODER_TICK_H)
									.get(Distance.Unit.INCH));
	
		}
	}

	@Override
	public void onEnd() {
		Drive.getInstance().disableProfiler();
		Drive.getInstance().setMotorSpeedInPercent(0, 0, 0);
	}

	@Override
	public boolean isFinishedNR() {
		
		boolean finished;
		
		finished = Math.abs((((Math.abs(Drive.getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
				- initialLeftPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
				+ Drive.getInstance().getRightPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
				- initialRightPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE))) / 2)
				- Math.abs(HDriveDiagonalProfiler.posPoints
						.get(HDriveDiagonalProfiler.posPoints.size() - 1)))) < Drive.END_THRESHOLD
								.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
				&& /*Math.abs(((Math.abs(Drive.getInstance().getHPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H)
						- initialHPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H)))
						- Math.abs(HDriveDiagonalProfiler.posPointsH
								.get(HDriveDiagonalProfiler.posPointsH.size() - 1)))) < Drive.END_THRESHOLD
										.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H)
				&&*/ Drive.getInstance().getLeftVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD)
				&& Drive.getInstance().getRightVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD);
				//&& Drive.getInstance().getHVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD);
		
		System.out.println(Drive.getInstance().getLeftVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD)
				&& Drive.getInstance().getRightVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD));
		
		return finished;
	}

}
