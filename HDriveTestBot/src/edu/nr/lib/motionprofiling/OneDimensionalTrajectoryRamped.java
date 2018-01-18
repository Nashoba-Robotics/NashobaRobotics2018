package edu.nr.lib.motionprofiling;

import java.util.ArrayList;

import edu.nr.lib.NRMath;

public class OneDimensionalTrajectoryRamped implements OneDimensionalTrajectory {

	double velMax;
	double accelMax;

	double velMaxUsed;
	double accelMaxUsed;

	double totalTime;
	double timeRamp;
	double timeAccel;
	double timeCruise;

	double endPosition;
	double startPosition;

	double pow = 2;
	double timeMult = 5;

	ArrayList<Double> posPoints;
	ArrayList<Double> velPoints;
	ArrayList<Double> accelPoints;

	double direction;

	/**
	 * 
	 * @param goalPositionDelta
	 *            The goal position that can be either positive or negative
	 * @param velMax
	 *            The max velocity (always positive)
	 * @param accelMax
	 *            The max acceleration (always positive)
	 */
	public OneDimensionalTrajectoryRamped(double goalPositionDelta, double velMax, double accelMax) {
		this.endPosition = Math.abs(goalPositionDelta);
		this.startPosition = 0;
		this.direction = Math.signum(goalPositionDelta);

		this.velMax = Math.abs(velMax);
		this.accelMax = Math.abs(accelMax);

		posPoints = new ArrayList<Double>();
		velPoints = new ArrayList<Double>();
		accelPoints = new ArrayList<Double>();

		calcTimes();
	}

	/**
	 * Calculates times of timeRamp, timeAccel, and timeCruise
	 */
	private void calcTimes() {
		velMaxUsed = velMax;
		accelMaxUsed = accelMax;
		timeRamp = flipDerivRampFunc(accelMax);
		double dVr = rampFunc(timeRamp);
		timeAccel = (velMaxUsed - (2 * dVr)) / accelMax;
		timeCruise = (endPosition - 2 * integRampFunc(0, timeRamp) - 2 * timeAccel * dVr
				- timeAccel * (velMaxUsed - 2 * dVr) - 2 * timeRamp * (velMaxUsed - dVr)
				- 2 * integXYRefRampFunc(0, timeRamp)) / velMaxUsed;
		if (timeCruise <= 0 || timeAccel <= 0) {
			timeAccel = NRMath.quadratic(accelMax,
					2 * timeRamp * accelMax + 2 * rampFunc(timeRamp), 2 * integRampFunc(0, timeRamp)
							+ 2 * integXYRefRampFunc(0, timeRamp) + 2 * timeRamp * rampFunc(timeRamp) - endPosition,
					true);
			timeCruise = 0;
			velMaxUsed = 2 * rampFunc(timeRamp) + timeAccel * accelMax;
		}
		if (timeAccel < 0) {
			timeRamp = Math.pow(endPosition / (4 * timeMult), 1 / (pow + 1));
			timeAccel = 0;
			velMaxUsed = 2 * rampFunc(timeRamp);
			accelMaxUsed = derivRampFunc(timeRamp);
		}

		totalTime = 4 * timeRamp + 2 * timeAccel + timeCruise;
	}

	public double rampFunc(double time) {
		return timeMult * Math.pow(time, pow);
	}

	public double xyRefRampFunc(double time) {
		return -rampFunc(timeRamp - time) + rampFunc(timeRamp);
	}

	public double integRampFunc(double time1, double time2) {
		return timeMult / (pow + 1) * Math.pow(time2, pow + 1) - timeMult / (pow + 1) * Math.pow(time1, pow + 1);
	}

	public double integXYRefRampFunc(double time1, double time2) {
		return rampFunc(timeRamp) * (time2 - time1) - (timeMult / (pow + 1) * Math.pow(timeRamp - time1, pow + 1)
				- timeMult / (pow + 1) * Math.pow(timeRamp - time2, pow + 1));
	}

	/**
	 * 
	 * @param time
	 * @return accel at specific time
	 */
	public double derivRampFunc(double time) {
		return timeMult * pow * Math.pow(time, pow - 1);
	}

	/**
	 * 
	 * @param accel
	 * @return time when derivative equals acceleration
	 */
	public double flipDerivRampFunc(double accel) {
		return Math.pow(accel / (pow * timeMult), 1.0 / (pow - 1));
	}

	/**
	 * 
	 * @param time
	 * @return accel at specific time
	 */
	public double derivXYRefRampFunc(double time) {
		return (timeMult * pow * Math.pow(timeRamp - time, pow - 1));
	}

	public double getGoalVelocity(double time) {
		if (time < timeRamp) {
			return (rampFunc(time)) * direction;
		} else if (time < timeRamp + timeAccel) {
			return (rampFunc(timeRamp) + accelMaxUsed * (time - timeRamp)) * direction;
		} else if (time < 2 * timeRamp + timeAccel) {
			return (rampFunc(timeRamp) + (accelMaxUsed * timeAccel) + xyRefRampFunc(time - timeAccel - timeRamp))
					* direction;
		} else if (time < 2 * timeRamp + timeAccel + timeCruise) {
			return (velMaxUsed) * direction;
		} else if (time < 3 * timeRamp + timeAccel + timeCruise) {
			return (xyRefRampFunc((timeCruise + 3 * timeRamp + timeAccel) - time) + (accelMaxUsed * timeAccel)
					+ rampFunc(timeRamp)) * direction;
		} else if (time < 3 * timeRamp + 2 * timeAccel + timeCruise) {
			return (-accelMaxUsed * (time - 3 * timeRamp - timeAccel - timeCruise) + rampFunc(timeRamp)
					+ (accelMaxUsed * timeAccel)) * direction;
		} else if (time < 4 * timeRamp + 2 * timeAccel + timeCruise) {
			return (rampFunc(timeRamp * 4 + timeAccel * 2 + timeCruise - time)) * direction;
		} else {
			return 0;
		}
	}

	public double getGoalPosition(double time) {
		if (time < timeRamp) {
			return (integRampFunc(0, time)) * direction;
		} else if (time < timeRamp + timeAccel) {
			return (integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (time - timeRamp)
					+ 0.5 * (time - timeRamp)
							* ((rampFunc(timeRamp) + accelMaxUsed * (time - timeRamp)) - rampFunc(timeRamp)))
					* direction;
		} else if (time < 2 * timeRamp + timeAccel) {
			return (integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel)
					+ 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMaxUsed * (timeAccel)) - rampFunc(timeRamp))
					+ integXYRefRampFunc(0, time - timeAccel - timeRamp)
					+ (rampFunc(timeRamp) + timeAccel * accelMaxUsed) * (time - timeAccel - timeRamp)) * direction;
		} else if (time < 2 * timeRamp + timeAccel + timeCruise) {
			return (integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel)
					+ 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMaxUsed * (timeAccel)) - rampFunc(timeRamp))
					+ integXYRefRampFunc(0, timeRamp) + (velMaxUsed - xyRefRampFunc(timeRamp)) * (timeRamp)
					+ velMaxUsed * (time - (2 * timeRamp + timeAccel))) * direction;
		} else if (time < 3 * timeRamp + timeAccel + timeCruise) {
			return (integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel)
					+ 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMaxUsed * (timeAccel)) - rampFunc(timeRamp))
					+ integXYRefRampFunc(0, timeRamp) + (velMaxUsed - xyRefRampFunc(timeRamp)) * (timeRamp)
					+ velMaxUsed * (timeCruise) + integXYRefRampFunc(0, timeRamp)
					- integXYRefRampFunc(0, (3 * timeRamp) + timeCruise + timeAccel - time)
					+ (velMaxUsed - xyRefRampFunc(timeRamp)) * (time - timeCruise - (2 * timeRamp) - timeAccel))
					* direction;
		} else if (time < 3 * timeRamp + 2 * timeAccel + timeCruise) {
			return (integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel)
					+ 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMaxUsed * (timeAccel)) - rampFunc(timeRamp))
					+ integXYRefRampFunc(0, timeRamp) + (velMaxUsed - xyRefRampFunc(timeRamp)) * (timeRamp)
					+ velMaxUsed * (timeCruise) + integXYRefRampFunc(0, timeRamp)
					+ (velMaxUsed - xyRefRampFunc(timeRamp)) * (timeRamp)
					+ rampFunc(timeRamp) * (time - (3 * timeRamp) - timeAccel - timeCruise)
					- 0.5 * (((3 * timeRamp) + (2 * timeAccel) + timeCruise - time)
							* ((-accelMaxUsed * (time - 3 * timeRamp - timeAccel - timeCruise))
									+ (accelMaxUsed * timeAccel)))
					+ 0.5 * (timeAccel) * (velMaxUsed - (2 * rampFunc(timeRamp)))) * direction;
		} else if (time < 4 * timeRamp + 2 * timeAccel + timeCruise) {
			return (integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel)
					+ 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMaxUsed * (timeAccel)) - rampFunc(timeRamp))
					+ integXYRefRampFunc(0, timeRamp) + (velMaxUsed - xyRefRampFunc(timeRamp)) * (timeRamp)
					+ velMaxUsed * (timeCruise) + integXYRefRampFunc(0, timeRamp)
					+ (velMaxUsed - xyRefRampFunc(timeRamp)) * (timeRamp) + rampFunc(timeRamp) * (timeAccel)
					+ 0.5 * (timeAccel) * (velMaxUsed - (2 * rampFunc(timeRamp))) + integRampFunc(0, timeRamp)
					- integRampFunc(0, ((4 * timeRamp) + (2 * timeAccel) + timeCruise - time))) * direction;
		} else {
			return (endPosition) * direction;
		}
	}

	public double getGoalAccel(double time) {
		if (time < timeRamp) {
			return (derivRampFunc(time)) * direction;
		} else if (time < timeRamp + timeAccel) {
			return (accelMaxUsed) * direction;
		} else if (time < (2 * timeRamp) + timeAccel) {
			return (derivXYRefRampFunc(time - (timeAccel + timeRamp))) * direction;
		} else if (time < (2 * timeRamp) + timeAccel + timeCruise) {
			return 0;
		} else if (time < (3 * timeRamp) + timeAccel + timeCruise) {
			return (-derivXYRefRampFunc(((3 * timeRamp) + timeAccel + timeCruise) - time)) * direction;
		} else if (time < (3 * timeRamp) + (2 * timeAccel) + timeCruise) {
			return (-accelMaxUsed) * direction;
		} else if (time < 4 * timeRamp + 2 * timeAccel + timeCruise) {
			return (-derivRampFunc(((timeRamp * 4) + (timeAccel * 2) + timeCruise) - time)) * direction;
		} else {
			return 0;
		}
	}

	/**
	 * Multiply totalTime by 100 to get milliseconds per hundred ms
	 */
	@Override
	public ArrayList<Double> loadPosPoints(double period) {
		posPoints.clear();
		for (int time = 0; time < Math.round(totalTime * 100); time += period) {
			posPoints.add(getGoalPosition(time / 100.0));
		}
		return posPoints;
	}

	/**
	 * Multiply totalTime by 100 to get milliseconds per hundred ms
	 */
	@Override
	public ArrayList<Double> loadVelPoints(double period) {
		velPoints.clear();
		for (int time = 0; time < Math.round(totalTime * 100); time += period) {
			velPoints.add(getGoalVelocity(time / 100.0));
		}
		return velPoints;
	}

	/**
	 * Multiply totalTime by 100 to get milliseconds per hundred ms
	 */
	@Override
	public ArrayList<Double> loadAccelPoints(double period) {
		accelPoints.clear();
		for (int time = 0; time < Math.round(totalTime * 100); time += period) {
			accelPoints.add(getGoalAccel(time / 100.0));
		}
		return accelPoints;
	}

	@Override
	public double getMaxUsedAccel() {
		return accelMaxUsed;
	}

	@Override
	public double getEndPosition() {
		return endPosition;
	}

	@Override
	public double getGoalHeading(double time) {
		return 0;
	}

	@Override
	public double getMaxUsedVelocity() {
		return velMaxUsed;
	}

}