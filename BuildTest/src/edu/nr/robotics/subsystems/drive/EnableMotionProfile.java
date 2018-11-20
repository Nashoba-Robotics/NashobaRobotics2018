package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotor;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableMotionProfile extends NRCommand {
	
	Distance initialLeftPosition;
	Distance initialRightPosition;
	Distance tempLeftPosition = Distance.ZERO;
	Distance tempRightPosition = Distance.ZERO;
	
	Distance distX;
	Distance distY;
	Angle endAngle;
	double maxVelPercent;
	double maxAccelPercent;
	String profileName;
		
	public EnableMotionProfile(Distance distX, Distance distY, Angle endAngle, double maxVelPercent, double maxAccelPercent, String profileName) {
		super(Drive.getInstance());
		this.distX = distX;
		this.distY = distY;
		this.endAngle = endAngle;
		this.maxVelPercent = maxVelPercent;
		this.maxAccelPercent = maxAccelPercent;
		this.profileName = profileName;
	}
	
	@Override
	public void onStart() {
		Drive.getInstance().enableMotionProfiler(distX, distY, endAngle, maxVelPercent, maxAccelPercent, profileName);
		initialLeftPosition = Drive.getInstance().getLeftPosition();
		initialRightPosition = Drive.getInstance().getRightPosition();
	}

	@Override
	public void onExecute() {
			Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
			SmartDashboard.putNumberArray("MotionProfiler V Left", new double[] {Drive.getInstance().pidGetLeft(), OneDimensionalMotionProfilerTwoMotor.velocityGoal});
			SmartDashboard.putNumberArray("Motion Profiler V Right", new double[] {Drive.getInstance().pidGetRight(), OneDimensionalMotionProfilerTwoMotor.velocityGoal});
			
			/*SmartDashboard.putString("Motion Profiler V Left",
					Drive.getInstance().pidGetLeft() + ":" + OneDimensionalMotionProfilerTwoMotor.velocityGoal);
			SmartDashboard.putString("Motion Profiler V Right",
					Drive.getInstance().pidGetRight() + ":" + OneDimensionalMotionProfilerTwoMotor.velocityGoal);*/
		
			Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
			SmartDashboard.putNumberArray("Motion Profiler X Left", new double[] {new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
					.get(Distance.Unit.INCH), 
					new Distance(
							OneDimensionalMotionProfilerTwoMotor.positionGoal + OneDimensionalMotionProfilerTwoMotor.initialPositionLeft,
							Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH),
					new Distance(OneDimensionalMotionProfilerTwoMotor.errorLeft, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
					.get(Distance.Unit.INCH)});
			SmartDashboard.putNumberArray("Motion Profiler X Right", new double[] {new Distance(Drive.getInstance().pidGetRight(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
					.get(Distance.Unit.INCH), 
					new Distance(
							OneDimensionalMotionProfilerTwoMotor.positionGoal + OneDimensionalMotionProfilerTwoMotor.initialPositionRight,
							Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH),
					new Distance(OneDimensionalMotionProfilerTwoMotor.errorRight, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
					.get(Distance.Unit.INCH)});
			
			/*SmartDashboard.putString("Motion Profiler X Left",
					new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
							.get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(
									OneDimensionalMotionProfilerTwoMotor.positionGoal + OneDimensionalMotionProfilerTwoMotor.initialPositionLeft,
									Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(OneDimensionalMotionProfilerTwoMotor.errorLeft, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
									.get(Distance.Unit.INCH));
			SmartDashboard.putString("Motion Profiler X Right",
					new Distance(Drive.getInstance().pidGetRight(), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
							.get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(
									OneDimensionalMotionProfilerTwoMotor.positionGoal + OneDimensionalMotionProfilerTwoMotor.initialPositionRight,
									Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE).get(Distance.Unit.INCH)
							+ ":"
							+ new Distance(OneDimensionalMotionProfilerTwoMotor.errorRight, Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
									.get(Distance.Unit.INCH));*/
		
	}

	@Override
	public void onEnd() {
		Drive.getInstance().disableProfiler();
		Drive.getInstance().setMotorSpeedInPercent(0, 0);
	}

	@Override
	public boolean isFinishedNR() {
		
		boolean finished;
		
		finished = Math.abs((((Math.abs(Drive.getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
				- initialLeftPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
				+ Drive.getInstance().getRightPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
				- initialRightPosition.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE))) / 2)
				- Math.abs(OneDimensionalMotionProfilerTwoMotor.posPoints
						.get(OneDimensionalMotionProfilerTwoMotor.posPoints.size() - 1)))) < Drive.END_THRESHOLD
								.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)
								&& Drive.getInstance().getLeftVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD)
				&& Drive.getInstance().getRightVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD);		
		return finished;
	}

}
