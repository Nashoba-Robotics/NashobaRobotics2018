package edu.nr.lib.motionprofiling;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OneDimensionalMotionProfilerBasic extends TimerTask implements OneDimensionalMotionProfiler  {

	private final Timer timer;
	
	//In milliseconds
	private final long period;
	private static final long defaultPeriod = 20; //50 Hz 
	
	private double prevTime;
	private double startTime;
	
	public double positionGoal;
	public double velocityGoal;
	public double accelGoal;
	
	private boolean enabled = false;
	private PIDOutput out;
	private PIDSource source;
	
	private double ka, kp, kd, kv;
	public double errorLast;
	public double error;
	
	public double initialPosition;
	
	public ArrayList<Double> posPoints;
	public ArrayList<Double> velPoints;
	public ArrayList<Double> accelPoints;
	
	private OneDimensionalTrajectory trajectory;
	
	public int loopIteration;
		
	public OneDimensionalMotionProfilerBasic(PIDOutput out, PIDSource source, double kv, double ka, double kp, double kd, long period) {
		this.out = out;
		this.source = source;
		this.period = period;
		this.trajectory = new OneDimensionalTrajectorySimple(0,1,1);
		timer = new Timer();
		timer.scheduleAtFixedRate(this, 0, this.period);
		reset();
		this.source.setPIDSourceType(PIDSourceType.kDisplacement);
		this.ka = ka;
		this.kp = kp;
		this.kd = kd;
		this.kv = kv;
		posPoints = new ArrayList<Double>();
		velPoints = new ArrayList<Double>();
		accelPoints = new ArrayList<Double>();
		initialPosition = source.pidGet();
	}
	
	public OneDimensionalMotionProfilerBasic(PIDOutput out, PIDSource source, double kv, double ka, double kp, double kd) {
		this(out, source, kv, ka, kp, kd, defaultPeriod);
	}
	
	double timeOfVChange = 0;
	double prevV;
	
	@Override
	public void run() {
		if(enabled) {
			double dt = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - prevTime;

			if (loopIteration < posPoints.size()) {
				positionGoal = posPoints.get(loopIteration);
				velocityGoal = velPoints.get(loopIteration);
				accelGoal = accelPoints.get(loopIteration);
			} else {
				positionGoal = posPoints.get(posPoints.size() - 1);
				velocityGoal = 0;
				accelGoal = 0;
			}
						
			source.setPIDSourceType(PIDSourceType.kDisplacement);
			error = positionGoal - source.pidGet() + initialPosition;
									
			double errorDeriv = (error - errorLast) / dt;
			
			double output = velocityGoal * kv + error * kp + errorDeriv * kd;
			
			if ((accelGoal > 0 && velocityGoal > 0) || (accelGoal < 0 && velocityGoal < 0)) {
				output += accelGoal * ka;
			}
			
			source.setPIDSourceType(PIDSourceType.kRate);
			out.pidWrite(output);

			errorLast = error;
			
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
		reset();
		posPoints = trajectory.loadPosPoints(period);
		velPoints = trajectory.loadVelPoints(period);
		accelPoints = trajectory.loadAccelPoints(period);
		enabled = true;
	}
	
	/**
	 * Reset the previous time to the current time.
	 * Doesn't disable the controller
	 */
	public void reset() {
		errorLast = 0;
		startTime = prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		PIDSourceType type = source.getPIDSourceType();
		source.setPIDSourceType(PIDSourceType.kDisplacement);
		initialPosition = source.pidGet();
		loopIteration = 0;
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
