package edu.nr.lib.motionprofiling;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.interfaces.TriplePIDOutput;
import edu.nr.lib.interfaces.TriplePIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class OneDimensionalMotionProfilerHDriveMain extends TimerTask implements OneDimensionalMotionProfiler  {

	private final Timer timer;
	
	//In milliseconds
	private final long period;
	private static final long defaultPeriod = 10; //100 Hz 
	
	private double prevTime;
	private double startTime;
	
	private boolean enabled = false;
	private TriplePIDOutput out;
	private TriplePIDSource source;
	
	private double ka, kp, ki, kd, kv, kp_theta;
	public static double errorH;
	private double errorHLast;
	
	public static double positionGoal, velocityGoal, accelGoal;
	
	public static double initialPositionH, initialPositionLeft, initialPositionRight;
	
	GyroCorrection gyroCorrection;
	
	public static ArrayList<Double> posPoints;
	public static ArrayList<Double> velPoints;
	public static ArrayList<Double> accelPoints;

	private int loopIteration;
			
	private OneDimensionalTrajectory trajectory;
		
	public OneDimensionalMotionProfilerHDriveMain(TriplePIDOutput out, TriplePIDSource source, double kv, double ka, double kp, double ki, double kd, double kp_theta, long period) {
		this.out = out;
		this.source = source;
		this.period = period;
		this.trajectory = new OneDimensionalTrajectoryRamped(0,1,1);
		timer = new Timer();
		timer.schedule(this, 0, this.period);
		reset();
		this.source.setPIDSourceType(PIDSourceType.kDisplacement);
		this.ka = ka;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.kv = kv;
		this.kp_theta = kp_theta;
		this.initialPositionH = source.pidGetH();
		this.initialPositionLeft = source.pidGetLeft();
		this.initialPositionRight = source.pidGetRight();
		this.gyroCorrection = new GyroCorrection();
		this.posPoints = new ArrayList<Double>();
		this.velPoints = new ArrayList<Double>();
		this.accelPoints = new ArrayList<Double>();
		reset();
		timer.scheduleAtFixedRate(this, 0, this.period);
	}
	
	public OneDimensionalMotionProfilerHDriveMain(TriplePIDOutput out, TriplePIDSource source, double kv, double ka, double kp, double ki, double kd, double kp_theta) {
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
			
			if (loopIteration < posPoints.size()) {
				positionGoal = posPoints.get(loopIteration);
				velocityGoal = velPoints.get(loopIteration);
				accelGoal = accelPoints.get(loopIteration);
			} else {
				positionGoal = posPoints.get(posPoints.size() - 1);
				velocityGoal = 0;
				accelGoal = 0;
			}
			
			errorH = positionGoal - source.pidGetH() + initialPositionH;
			double errorDerivH = (errorH - errorHLast) / dt;
			double errorIntegralH = (errorH - errorHLast) * dt / 2;
			double outputH = velocityGoal * kv + accelGoal * ka + errorH * kp + errorIntegralH * ki + errorDerivH * kd;
			errorHLast = errorH;
			
			double headingAdjustment = gyroCorrection.getTurnValue(kp_theta);
			
			double outputLeft, outputRight;
			
			outputLeft = -headingAdjustment;
			outputRight = headingAdjustment;
			
			out.pidWrite(outputLeft, outputRight, outputH);			
			//source.setPIDSourceType(PIDSourceType.kRate);
			//SmartDashboard.putString("Motion Profiler V", source.pidGet() + ":" + (output * trajectory.getMaxUsedVelocity() * Math.signum(trajectory.getMaxUsedVelocity())));
			//source.setPIDSourceType(PIDSourceType.kDisplacement);
			//SmartDashboard.putString("Motion Profiler X", source.pidGet() + ":" 
			//		+ (initialPosition + (trajectory.getGoalPosition(edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime))));
		}
		
		prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
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
		errorHLast = 0;
		startTime = prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		PIDSourceType type = source.getPIDSourceType();
		source.setPIDSourceType(PIDSourceType.kDisplacement);
		initialPositionH = source.pidGetH();
		source.setPIDSourceType(type);
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
	
}
