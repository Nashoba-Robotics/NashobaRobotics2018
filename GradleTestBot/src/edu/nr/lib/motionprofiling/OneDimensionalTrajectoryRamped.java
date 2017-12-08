package edu.nr.lib.motionprofiling;

public class OneDimensionalTrajectoryRamped implements OneDimensionalTrajectory {
	
	double velMax;
	double accelMax;
	
	double totalTime;
	double timeRamp;
	double timeAccel;
	double timeCruise;
	
	double endPosition;
	double startPosition;
	
	double direction;
	
	/**
	 * 
	 * @param goalPositionDelta The goal position that can be either positive or negative
	 * @param velMax The max velocity (always positive)
	 * @param accelMax The max acceleration (always positive)
	 */
	public OneDimensionalTrajectoryRamped(double goalPositionDelta, double velMax, double accelMax) {
		this.endPosition = Math.abs(goalPositionDelta);
		this.startPosition = 0;
		this.direction = Math.signum(goalPositionDelta);
		
		this.velMax = velMax;
		this.accelMax = accelMax;
		
		calcTimes();
	}

	private void calcTimes() {
		timeRamp = derivRampFunc(accelMax);
		double dVr = rampFunc(timeRamp);
		timeAccel = (velMax - (2 * dVr)) / accelMax;
		timeCruise = endPosition - (2 * (dVr * timeAccel - (0.5 * timeAccel * (velMax - 2 * dVr)))) - (2 * integRampFunc(0, timeRamp)) - (2 * (integXYRefRampFunc(0, timeRamp) + timeRamp * (velMax - dVr)));
	}
	
	public double rampFunc(double time) {
		return 0;
	}
	
	public double xyRefRampFunc(double time) {
		return 0;
	}
	
	public double integRampFunc(double time1, double time2) {
		return 0;
	}
	
	public double integXYRefRampFunc(double time1, double time2) {
		return 0;
	}
	
	/**
	 * 
	 * @param accel
	 * @return time when derivative equals acceleration
	 */
	public double derivRampFunc(double accel) {
		return 0;
	}
	
	public double derivXYRefRampFunc(double accel) {
		return 0;
	}
	
	public double getGoalVelocity(double time) {
		if (time < timeRamp) {
			return rampFunc(time);
		} else if (time < timeRamp + timeAccel) {
			return rampFunc(timeRamp) + accelMax * (time - timeRamp);
		} else if (time < 2 * timeRamp + timeAccel) {
			return rampFunc(timeRamp) + (accelMax * timeAccel) + xyRefRampFunc(time - timeAccel - timeRamp);
		} else if (time < 2 * timeRamp + timeAccel + timeCruise) {
			return velMax;
		} else if (time < 3 * timeRamp + timeAccel + timeCruise) {
			return xyRefRampFunc((timeCruise + 3 * timeRamp + timeAccel) - time) + (accelMax * timeAccel) + rampFunc(timeRamp);
		} else if (time < 3 * timeRamp + 2 * timeAccel + timeCruise) {
			return -accelMax * (time - 3 * timeRamp - timeAccel - timeCruise) + rampFunc(timeRamp);
		} else if (time < 4 * timeRamp + 2 * timeAccel + timeCruise) {
			return rampFunc(timeRamp * 4 + timeAccel * 2 + timeCruise - time);
		} else {
			return 0;
		}
	}
	
	public double getGoalPosition(double time) {	
		if (time < timeRamp) {
			return integRampFunc(0, time);
		} else if (time < timeRamp + timeAccel) {
			return integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (time - timeRamp) + 0.5 * (time-timeRamp) * ((rampFunc(timeRamp) + accelMax * (time - timeRamp)) - rampFunc(timeRamp));
		} else if (time < 2 * timeRamp + timeAccel) {
			return integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel) + 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMax * (timeAccel)) - rampFunc(timeRamp)) + integXYRefRampFunc(0, time - timeAccel - timeRamp) + (velMax - xyRefRampFunc(timeRamp))*(time - timeAccel - timeRamp);
		} else if (time < 2 * timeRamp + timeAccel + timeCruise) {
			return integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel) + 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMax * (timeAccel)) - rampFunc(timeRamp)) + integXYRefRampFunc(0, timeRamp) + (velMax - xyRefRampFunc(timeRamp))*(timeRamp) + velMax*(time - (2 * timeRamp + timeAccel));
		} else if (time < 3 * timeRamp + timeAccel + timeCruise) {
			return integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel) + 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMax * (timeAccel)) - rampFunc(timeRamp)) + integXYRefRampFunc(0, timeRamp) + (velMax - xyRefRampFunc(timeRamp))*(timeRamp) + velMax*(timeCruise) + integXYRefRampFunc(0, timeRamp) - integXYRefRampFunc(0, (3 * timeRamp) + timeCruise + timeAccel - time) + (velMax - xyRefRampFunc(timeRamp))*(time - timeCruise - (2 * timeRamp) - timeAccel);
		} else if (time < 3 * timeRamp + 2 * timeAccel + timeCruise) {
			return integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel) + 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMax * (timeAccel)) - rampFunc(timeRamp)) + integXYRefRampFunc(0, timeRamp) + (velMax - xyRefRampFunc(timeRamp))*(timeRamp) + velMax*(timeCruise) + integXYRefRampFunc(0, timeRamp) + (velMax - xyRefRampFunc(timeRamp))*(timeRamp) + rampFunc(timeRamp)*(time - (3 * timeRamp) - timeAccel - timeCruise) - 0.5 * (((3 * timeRamp) + (2 * timeAccel) + timeCruise - time)*((-accelMax * (time - 3 * timeRamp - timeAccel - timeCruise)) + rampFunc(timeRamp) - rampFunc(timeRamp))) + 0.5*(timeAccel)*(velMax - (2 * rampFunc(timeRamp)));
		} else if (time < 4 * timeRamp + 2 * timeAccel + timeCruise) {
			return integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel) + 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMax * (timeAccel)) - rampFunc(timeRamp)) + integXYRefRampFunc(0, timeRamp) + (velMax - xyRefRampFunc(timeRamp))*(timeRamp) + velMax*(timeCruise) + integXYRefRampFunc(0, timeRamp) + (velMax - xyRefRampFunc(timeRamp))*(timeRamp) + rampFunc(timeRamp)*(timeAccel) + 0.5*(timeAccel)*(velMax - (2 * rampFunc(timeRamp))) + integRampFunc(0, timeRamp) - integRampFunc(0, ((4 * timeRamp) + (2*timeAccel) + timeCruise - time));
		} else {
			return endPosition;
		}
	}

	public double getGoalAccel(double time) {
		if (time < timeRamp) {
			return derivRampFunc(time);
		} else if (time < timeRamp + timeAccel) {
			return accelMax;
		} else if (time < (2 * timeRamp) + timeAccel) {
			return derivXYRefRampFunc(time - (timeAccel + timeRamp));
		} else if (time < (2 * timeRamp) + timeAccel + timeCruise) {
			return 0;
		} else if (time < (3 * timeRamp) + timeAccel + timeCruise) {
			return derivXYRefRampFunc(((3 * timeRamp) + timeAccel + timeCruise) - time);
		} else if (time < (3 * timeRamp) + (2 * timeAccel) + timeCruise) {
			return -accelMax;
		} else if (time < 4 * timeRamp + 2 * timeAccel + timeCruise) {
			return derivRampFunc(((timeRamp * 4) + (timeAccel * 2) + timeCruise) - time);
		} else {
			return 0;
		}
	}

	@Override
	public double getMaxUsedAccel() {
		return accelMax;
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
		return velMax;
	}

	
}