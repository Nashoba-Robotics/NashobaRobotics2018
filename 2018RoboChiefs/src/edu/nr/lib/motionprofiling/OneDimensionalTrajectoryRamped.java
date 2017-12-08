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
		
	public OneDimensionalTrajectoryRamped(double goalPositionDelta, double velMax, double accelMax) {
		this.endPosition = goalPositionDelta;
		this.startPosition = 0;
		if(goalPositionDelta < 0) {
			velMax *= -1;
			accelMax *= -1; 
		}
		
		this.velMax = velMax;
		this.accelMax = accelMax;
		
		calcTimes();
	}

	private void calcTimes() {
		timeRamp = derivRampFunc(Math.abs(accelMax));
		double dVr = rampFunc(timeRamp);
		timeAccel = (velMax - (2 * dVr)) / accelMax;
		timeCruise = Math.abs(endPosition - (2 * (dVr * timeAccel - (0.5 * timeAccel * (Math.abs(velMax) - 2 * dVr)))) - (2 * integRampFunc(0, timeRamp)) - (2 * (integXYRefRampFunc(0, timeRamp) + timeRamp * (Math.abs(velMax) - dVr))));
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
			return  xyRefRampFunc((timeCruise + 3 * timeRamp + timeAccel) - time) + (accelMax * timeAccel) + rampFunc(timeRamp);
		} else if (time < 3 * timeRamp + 2 * timeAccel + timeCruise) {
			return -accelMax * (time - 3 * timeRamp - timeAccel - timeCruise) + rampFunc(timeRamp);
		} else if (time < 4 * timeRamp + 2 * timeAccel + timeCruise) {
			return rampFunc(timeRamp * 4 + timeAccel * 2 + timeCruise - time);
		} else {
			return 0;
		}
	}
	
	public double getGoalPosition(double time) {	
		return 0;
	}

	public double getGoalAccel(double time) {
		return 0;
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