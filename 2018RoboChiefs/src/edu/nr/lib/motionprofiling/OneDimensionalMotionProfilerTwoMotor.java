package edu.nr.lib.motionprofiling;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class OneDimensionalMotionProfilerTwoMotor extends TimerTask implements OneDimensionalMotionProfiler, SmartDashboardSource {

	private final Timer timer;
	
	//In milliseconds
	private final long period;
	private static final long defaultPeriod = 10; //200 Hz 
	
	private double prevTime;
	private double startTime;
	
	private boolean enabled = true;
	private DoublePIDOutput out;
	private DoublePIDSource source;
	
	private double ka, kp, ki, kd, kv, kp_theta;
	private double errorLastLeft;
	private double errorLastRight;
	
	private double initialPositionLeft;
	private double initialPositionRight;
			
	private OneDimensionalTrajectory trajectory;
	
	GyroCorrection gyroCorrection;
	
	private ArrayList<Double> posPoints;
	private ArrayList<Double> velPoints;
	private ArrayList<Double> accelPoints;

	private int loopIteration;
	
	public OneDimensionalMotionProfilerTwoMotor(DoublePIDOutput out, DoublePIDSource source, double kv, double ka, double kp, double ki, double kd, double kp_theta, long period) {
		this.out = out;
		this.source = source;
		this.period = period;
		this.trajectory = new OneDimensionalTrajectorySimple(0,1,1);
		timer = new Timer();
		this.source.setPIDSourceType(PIDSourceType.kDisplacement);
		this.ka = ka;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.kv = kv;
		this.kp_theta = kp_theta;
		this.initialPositionLeft = source.pidGetLeft();
		this.initialPositionRight = source.pidGetRight();
		this.gyroCorrection = new GyroCorrection();
		this.posPoints = new ArrayList<Double>();
		this.velPoints = new ArrayList<Double>();
		this.accelPoints = new ArrayList<Double>();
		reset();
		timer.scheduleAtFixedRate(this, 0, this.period);
	}
	
	public OneDimensionalMotionProfilerTwoMotor(DoublePIDOutput out, DoublePIDSource source, double kv, double ka, double kp, double ki, double kd, double kp_theta) {
		this(out, source, kv, ka, kp, ki, kd, kp_theta, defaultPeriod);
	}
	
	double timeOfVChange = 0;
	double prevV;
	
	@Override
	public void run() {
		if(enabled && posPoints.size() > 0 && velPoints.size() > 0 && accelPoints.size() > 0) {
			double dt = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - prevTime;
			prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
			//System.out.println(dt * 1000);
			
			double positionGoal;
			double velocityGoal;
			double accelGoal;

			if (loopIteration < posPoints.size()) {
				positionGoal = posPoints.get(loopIteration);
				velocityGoal = velPoints.get(loopIteration);
				accelGoal = accelPoints.get(loopIteration);
			} else {
				positionGoal = posPoints.get(posPoints.size() - 1);
				velocityGoal = 0;
				accelGoal = 0;
			}
			
			/*
			double currentTimeSinceStart = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime;

			double velocityGoal = trajectory.getGoalVelocity(currentTimeSinceStart);
			double positionGoal = trajectory.getGoalPosition(currentTimeSinceStart);
			double accelGoal = trajectory.getGoalAccel(currentTimeSinceStart);
			*/
			
			double headingAdjustment = gyroCorrection.getTurnValue(kp_theta);
			
			double errorLeft = positionGoal - source.pidGetLeft() + initialPositionLeft;			
			double errorDerivLeft = (errorLeft - errorLastLeft) / dt;
			double errorIntegralLeft = (errorLeft - errorLastLeft) * dt / 2;
			double prelimOutputLeft = velocityGoal * kv + accelGoal * ka + errorLeft * kp + errorIntegralLeft * ki + errorDerivLeft * kd;
			errorLastLeft = errorLeft;
			
			double outputLeft = 0;

			if (prelimOutputLeft > 0.0) {
				if (headingAdjustment > 0.0) {
					outputLeft = prelimOutputLeft - headingAdjustment;
				} else {
					outputLeft = Math.max(prelimOutputLeft, -headingAdjustment);
				}
			} else {
				if (headingAdjustment > 0.0) {
					outputLeft = -Math.max(-prelimOutputLeft, headingAdjustment);
				} else {
					outputLeft = prelimOutputLeft - headingAdjustment;
				}
			}
			
			double errorRight = positionGoal - source.pidGetRight() + initialPositionRight;			
			double errorDerivRight = (errorRight - errorLastRight) / dt;
			double prelimOutputRight = velocityGoal * kv + accelGoal * ka + errorRight * kp + errorDerivRight * kd;
			errorLastRight = errorRight;
			
			double outputRight = 0;
			
			if (prelimOutputRight > 0.0) {
				if (headingAdjustment > 0.0) {
					outputRight = Math.max(prelimOutputRight, headingAdjustment);
				} else {
					outputRight = prelimOutputRight + headingAdjustment;
				}
			} else {
				if (headingAdjustment > 0.0) {
					outputRight = prelimOutputRight + headingAdjustment;
				} else {
					outputRight = -Math.max(-prelimOutputRight, -headingAdjustment);
				}
			}
			
			out.pidWrite(outputLeft, outputRight);
			
			loopIteration++;
		}
	}
		
	/**
	 * Stop the profiler from running and resets it
	 */
	public void disable() {
		enabled = false;
		reset();
	}
	
	/**
	 * Reset the profiler and start it running
	 */
	public void enable() {
		//System.out.println("enabled");
		reset();
		posPoints = trajectory.loadPosPoints(period);
		System.out.println(posPoints.size());
		velPoints = trajectory.loadVelPoints(period);
		accelPoints = trajectory.loadAccelPoints(period);
		enabled = true;
	}
	
	/**
	 * Reset the previous time to the current time.
	 * Doesn't disable the controller
	 */
	public void reset() {
		errorLastLeft = 0;
		errorLastRight = 0;
		startTime = prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		PIDSourceType type = source.getPIDSourceType();
		source.setPIDSourceType(PIDSourceType.kDisplacement);
		initialPositionLeft = source.pidGetLeft();
		initialPositionRight = source.pidGetRight();
		source.setPIDSourceType(type);
		gyroCorrection.clearInitialValue();
		loopIteration = 0;
	}
	
	/**
	 * Sets the trajectory for the profiler
	 * @param trajectory
	 */
	public void setTrajectory(OneDimensionalTrajectory trajectory) {
		this.trajectory = trajectory;
	}

	@Override
	public boolean isEnabled() {
		return enabled;
	}

	@Override
	public OneDimensionalTrajectory getTrajectory() {
		return trajectory;
	}
	
	public void setKA(double ka) {
		this.ka = ka;
	}
	
	public void setKP(double kp) {
		this.kp = kp;
	}
	
	public void setKD(double kd) {
		this.kd = kd;
	}

	public void setKV(double kv) {
		this.kv = kv;
	}

	public void setKP_theta(double kp_theta) {
		this.kp_theta = kp_theta;
	}

	@Override
	public void smartDashboardInfo() {
		//source.setPIDSourceType(PIDSourceType.kRate);
		//SmartDashboard.putString("Motion Profiler V Left", source.pidGetLeft() + ":" + outputLeft * trajectory.getMaxPossibleVelocity() * Math.signum(trajectory.getMaxPossibleVelocity()) + ":" + Drive.getInstance().leftMotorSetpoint);
		//SmartDashboard.putString("Motion Profiler V Right", source.pidGetRight() + ":" + outputRight * trajectory.getMaxPossibleVelocity() * Math.signum(trajectory.getMaxPossibleVelocity()) + ":" + Drive.getInstance().rightMotorSetpoint);
		//SmartDashboard.putString("Motion Profiler V Left", source.pidGetLeft() + ":" + outputLeft * trajectory.getMaxPossibleVelocity() * Math.signum(trajectory.getMaxPossibleVelocity()));
		//SmartDashboard.putString("Motion Profiler V Right", source.pidGetRight() + ":" + outputRight * trajectory.getMaxPossibleVelocity() * Math.signum(trajectory.getMaxPossibleVelocity()));
		//SmartDashboard.putString("Motion Profiler V Left", source.pidGetLeft() + ":" + velocityGoal);
		//SmartDashboard.putString("Motion Profiler V Right", source.pidGetRight() + ":" + velocityGoal);
		//source.setPIDSourceType(PIDSourceType.kDisplacement);
		//SmartDashboard.putString("Motion Profiler X Left", source.pidGetLeft() + ":" + (positionGoal + initialPositionLeft) + ":" + errorLeft);
		//SmartDashboard.putString("Motion Profiler X Right", source.pidGetRight() + ":" + (positionGoal + initialPositionRight) + ":" + errorRight);
	}
}
