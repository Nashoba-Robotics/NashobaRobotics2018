package edu.nr.lib.motionprofiling;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.interfaces.TriplePIDOutput;
import edu.nr.lib.interfaces.TriplePIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class HDriveDiagonalProfiler extends TimerTask implements SmartDashboardSource {

	private final Timer timer;

	// In milliseconds
	private final long period;
	private static final long defaultPeriod = 20; // 200 Hz

	private double prevTime;
	private double startTime;

	private boolean enabled = true;
	private TriplePIDOutput out;
	private TriplePIDSource source;

	private double ka, kp, ki, kd, kv, kp_theta, ka_H, kp_H, ki_H, kd_H, kv_H;
	private double errorLastLeft;
	private double errorLastRight;
	private double errorLastH;

	public static double initialPositionLeft;
	public static double initialPositionRight;
	public static double initialPositionH;

	public static double positionGoal;
	public static double velocityGoal;
	public static double accelGoal;

	public static double positionGoalH;
	public static double velocityGoalH;
	public static double accelGoalH;

	public static double errorRight;
	public static double errorLeft;
	public static double errorH;

	private RampedDiagonalHTrajectory trajectory;

	GyroCorrection gyroCorrection;

	public static ArrayList<Double> posPoints;
	public static ArrayList<Double> velPoints;
	public static ArrayList<Double> accelPoints;

	public static ArrayList<Double> posPointsH;
	public static ArrayList<Double> velPointsH;
	public static ArrayList<Double> accelPointsH;

	private int loopIteration;

	public HDriveDiagonalProfiler(TriplePIDOutput out, TriplePIDSource source, double kv, double ka, double kp,
			double ki, double kd, double kp_theta, double kv_H, double ka_H, double kp_H, double ki_H, double kd_H,
			long period) {
		this.out = out;
		this.source = source;
		this.period = period;
		this.trajectory = new RampedDiagonalHTrajectory(0, 0, 1, 1);
		timer = new Timer();
		this.source.setPIDSourceType(PIDSourceType.kDisplacement);
		this.ka = ka;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.kv = kv;
		this.kp_theta = kp_theta;
		this.ka_H = ka_H;
		this.kp_H = kp_H;
		this.ki_H = ki_H;
		this.kd_H = kd_H;
		this.kv_H = kv_H;
		initialPositionLeft = source.pidGetLeft();
		initialPositionRight = source.pidGetRight();
		initialPositionH = source.pidGetH();
		gyroCorrection = new GyroCorrection();
		posPoints = new ArrayList<Double>();
		velPoints = new ArrayList<Double>();
		accelPoints = new ArrayList<Double>();
		posPointsH = new ArrayList<Double>();
		velPointsH = new ArrayList<Double>();
		accelPointsH = new ArrayList<Double>();
		reset();
		timer.scheduleAtFixedRate(this, 0, this.period);
	}

	public HDriveDiagonalProfiler(TriplePIDOutput out, TriplePIDSource source, double kv, double ka, double kp,
			double ki, double kd, double kp_theta, double kv_H, double ka_H, double kp_H, double ki_H, double kd_H) {
		this(out, source, kv, ka, kp, ki, kd, kp_theta, kv_H, ka_H, kp_H, ki_H, kd_H, defaultPeriod);
	}

	double timeOfVChange = 0;
	double prevV;

	public void smartDashboardInfo() {

	}

	public void disable() {
		enabled = false;
		reset();

	}

	public void enable() {
		reset();
		posPoints = trajectory.loadPosPoints(period);
		velPoints = trajectory.loadVelPoints(period);
		accelPoints = trajectory.loadAccelPoints(period);
		posPointsH = trajectory.loadPosPointsH(period);
		velPointsH = trajectory.loadVelPointsH(period);
		accelPointsH = trajectory.loadAccelPointsH(period);
		enabled = true;
		//System.out.println("trajectory loaded " + enabled);
	}

	public void reset() {
		errorLastLeft = 0;
		errorLastRight = 0;
		errorLastH = 0;
		PIDSourceType type = source.getPIDSourceType();
		source.setPIDSourceType(PIDSourceType.kDisplacement);
		initialPositionLeft = source.pidGetLeft();
		initialPositionRight = source.pidGetRight();
		initialPositionH = source.pidGetH();
		source.setPIDSourceType(type);
		gyroCorrection.clearInitialValue();
		loopIteration = 0;
	}

	public boolean isEnabled() {
		return enabled;
	}

	public OneDimensionalTrajectory getTrajectory() {
		return trajectory;
	}

	public void setTrajectory(RampedDiagonalHTrajectory trajectory) {
		this.trajectory = trajectory;
	}

	@Override
	public void run() {
		//System.out.println("is enabled: " + enabled);
		//System.out.println("position size: " + posPoints.size() + " vel size: " + velPoints.size() + " accel size: " + accelPoints.size() + " pos H size: " + posPointsH.size() + " vel H size: " + velPointsH.size() + " acc H size: " + accelPointsH.size());
		if (enabled && posPoints.size() > 0 && velPoints.size() > 0 && accelPoints.size() > 0 && posPointsH.size() > 0
				&& velPointsH.size() > 0 && accelPointsH.size() > 0) {
			double dt = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - prevTime;
			prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

			if (loopIteration < posPoints.size()) {
				positionGoal = posPoints.get(loopIteration);
				velocityGoal = velPoints.get(loopIteration);
				accelGoal = accelPoints.get(loopIteration);
			} else {
				positionGoal = posPoints.get(posPoints.size() - 1);
				velocityGoal = 0;
				accelGoal = 0;
			}

			if (loopIteration < posPointsH.size()) {
				positionGoalH = posPointsH.get(loopIteration);
				velocityGoalH = velPointsH.get(loopIteration);
				accelGoalH = accelPointsH.get(loopIteration);
			} else {
				positionGoalH = posPointsH.get(posPointsH.size() - 1);
				velocityGoalH = 0;
				accelGoalH = 0;
			}

			//System.out.println("Position Goal: " + positionGoal + "    Position Goal H: " + positionGoalH);
			
			double headingAdjustment = gyroCorrection.getTurnValue(kp_theta, false);

			source.setPIDSourceType(PIDSourceType.kDisplacement);
			errorLeft = (positionGoal - source.pidGetRight() + initialPositionRight + positionGoal - source.pidGetLeft() + initialPositionLeft) / 2;
			double errorDerivLeft = (errorLeft - errorLastLeft) / dt;
			double errorIntegralLeft = (errorLeft - errorLastLeft) * dt / 2;
			double prelimOutputLeft = velocityGoal * kv + accelGoal * ka + errorLeft * kp + errorIntegralLeft * ki
					+ errorDerivLeft * kd;
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

			source.setPIDSourceType(PIDSourceType.kDisplacement);
			errorRight = (positionGoal - source.pidGetRight() + initialPositionRight + positionGoal - source.pidGetLeft() + initialPositionLeft) / 2;
			double errorDerivRight = (errorRight - errorLastRight) / dt;
			double errorIntegralRight = (errorRight - errorLastRight) * dt / 2;
			double prelimOutputRight = velocityGoal * kv + accelGoal * ka + errorRight * kp + errorIntegralRight * ki 
					+ errorDerivRight * kd;
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

			source.setPIDSourceType(PIDSourceType.kDisplacement);
			errorH = positionGoalH - source.pidGetH() + initialPositionH;
			double errorDerivH = (errorH - errorLastH) / 2;
			double errorIntegralH = (errorH - errorLastH) * dt / 2;
			double outputH = velocityGoalH * kv_H + accelGoalH * ka_H + errorH * kp_H + errorIntegralH * ki_H
					+ errorDerivH * kd_H;
			errorLastH = errorH;
			
			out.pidWrite(outputLeft, outputRight, outputH);
			
			loopIteration++;
		}
	}
}
