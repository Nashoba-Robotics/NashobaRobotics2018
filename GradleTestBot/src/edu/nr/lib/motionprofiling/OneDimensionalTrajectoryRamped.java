package edu.nr.lib.motionprofiling;

import java.util.ArrayList;

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

	/**
	 * Calculates times of timeRamp, timeAccel, and timeCruise
	 */
	private void calcTimes() {
		timeRamp = flipDerivRampFunc(accelMax);
		double dVr = rampFunc(timeRamp);
		timeAccel = (velMax - (2 * dVr)) / accelMax;
		timeCruise = (endPosition - 2 * integRampFunc(0, timeRamp) - 2 * timeAccel * dVr - timeAccel * (velMax - 2 * dVr) - 2 * timeRamp * (velMax - dVr) - 2 * integXYRefRampFunc(0, timeRamp)) / velMax;
		totalTime = 4 * timeRamp + 2 * timeAccel + timeCruise;
	}
	
	public double rampFunc(double time) {
		return Math.pow(time, 4.0);
	}
	
	public double xyRefRampFunc(double time) {
		return -rampFunc(timeRamp - time) + rampFunc(timeRamp);
	}
	
	public double integRampFunc(double time1, double time2) {
		return 1.0 / 5.0 * Math.pow(time2, 5.0) - 1.0 / 5.0 * Math.pow(time1, 5.0);
	}
	
	public double integXYRefRampFunc(double time1, double time2) {
		return rampFunc(timeRamp) * (time2 - time1) - (1.0 / 5.0 * Math.pow(timeRamp - time2, 5.0) - 1.0 / 5.0 * Math.pow(timeRamp - time1, 5.0));
	}
	
	/**
	 * 
	 * @param time
	 * @return accel at specific time
	 */
	public double derivRampFunc(double time) {
		return 4.0 * Math.pow(time, 3.0);
		
	}
	
	/**
	 * 
	 * @param accel
	 * @return time when derivative equals acceleration
	 */
	public double flipDerivRampFunc(double accel) {
		return Math.pow(accel / 4.0, 1.0 / 3.0); //accel = 2 * time since velocity = x^2;
	}
	
	/**
	 * 
	 * @param time
	 * @return accel at specific time
	 */
	public double derivXYRefRampFunc(double time) {
		return (4 * Math.pow(timeRamp - time, 3));
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
			return -accelMax * (time - 3 * timeRamp - timeAccel - timeCruise) + rampFunc(timeRamp) + (accelMax * timeAccel);
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
			return integRampFunc(0, timeRamp) + 
					rampFunc(timeRamp) * (time - timeRamp) + 0.5 * (time-timeRamp) * ((rampFunc(timeRamp) + accelMax * (time - timeRamp)) - rampFunc(timeRamp));
		} else if (time < 2 * timeRamp + timeAccel) {
			return integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel) + 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMax * (timeAccel)) - rampFunc(timeRamp)) + integXYRefRampFunc(0, time - timeAccel - timeRamp) + (velMax - xyRefRampFunc(timeRamp))*(time - timeAccel - timeRamp);
		} else if (time < 2 * timeRamp + timeAccel + timeCruise) {
			return integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel) + 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMax * (timeAccel)) - rampFunc(timeRamp)) + integXYRefRampFunc(0, timeRamp) + (velMax - xyRefRampFunc(timeRamp))*(timeRamp) + velMax*(time - (2 * timeRamp + timeAccel));
		} else if (time < 3 * timeRamp + timeAccel + timeCruise) {
			return integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel) + 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMax * (timeAccel)) - rampFunc(timeRamp)) + integXYRefRampFunc(0, timeRamp) + (velMax - xyRefRampFunc(timeRamp))*(timeRamp) + velMax*(timeCruise) + integXYRefRampFunc(0, timeRamp) - integXYRefRampFunc(0, (3 * timeRamp) + timeCruise + timeAccel - time) + (velMax - xyRefRampFunc(timeRamp))*(time - timeCruise - (2 * timeRamp) - timeAccel);
		} else if (time < 3 * timeRamp + 2 * timeAccel + timeCruise) {
			return integRampFunc(0, timeRamp) + rampFunc(timeRamp) * (timeAccel) + 0.5 * (timeAccel) * ((rampFunc(timeRamp) + accelMax * (timeAccel)) - rampFunc(timeRamp)) + integXYRefRampFunc(0, timeRamp) + (velMax - xyRefRampFunc(timeRamp))*(timeRamp) + velMax*(timeCruise) + integXYRefRampFunc(0, timeRamp) + (velMax - xyRefRampFunc(timeRamp))*(timeRamp) + rampFunc(timeRamp)*(time - (3 * timeRamp) - timeAccel - timeCruise) - 0.5 * (((3 * timeRamp) + (2 * timeAccel) + timeCruise - time)*((-accelMax * (time - 3 * timeRamp - timeAccel - timeCruise)) + (accelMax * timeAccel))) + 0.5*(timeAccel)*(velMax - (2 * rampFunc(timeRamp)));
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
			return -derivXYRefRampFunc(((3 * timeRamp) + timeAccel + timeCruise) - time);
		} else if (time < (3 * timeRamp) + (2 * timeAccel) + timeCruise) {
			return -accelMax;
		} else if (time < 4 * timeRamp + 2 * timeAccel + timeCruise) {
			return -derivRampFunc(((timeRamp * 4) + (timeAccel * 2) + timeCruise) - time);
		} else {
			return 0;
		}
	}

	@Override
	public ArrayList<Double> loadPosPoints(double period) {
		ArrayList<Double> posPoints = new ArrayList<Double>();
		for (int time = 0; time < totalTime; time += period) {
			posPoints.add(getGoalPosition(time));
		}
		return posPoints;
	}
	
	@Override
	public ArrayList<Double> loadVelPoints(double period) {
		ArrayList<Double> velPoints = new ArrayList<Double>();
		for (int time = 0; time < totalTime; time += period) {
			velPoints.add(getGoalVelocity(time));
		}
		return velPoints;
	}
	
	@Override
	public ArrayList<Double> loadAccelPoints(double period) {
		ArrayList<Double> accelPoints = new ArrayList<Double>();
		for (int time = 0; time < totalTime; time += period) {
			accelPoints.add(getGoalVelocity(time));
		}
		return accelPoints;
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