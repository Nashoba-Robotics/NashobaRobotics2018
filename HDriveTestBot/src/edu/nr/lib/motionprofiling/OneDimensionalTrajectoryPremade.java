package edu.nr.lib.motionprofiling;

import java.util.ArrayList;

public class OneDimensionalTrajectoryPremade implements OneDimensionalTrajectory {

	// The values in the position, velocity, and acceleration lists
	// are offset by the period.
	// That is, the first value in the list is at time 0,
	// the second value in the list is at time $period,
	// the third value in the list is at time 2*$period,
	// and so on.	
	private final double period;
	private final ArrayList<Double> positionList;
	private final ArrayList<Double> velocityList;
	private final ArrayList<Double> accelerationList;
	
	public OneDimensionalTrajectoryPremade(ArrayList<Double> positionList, ArrayList<Double> velocityList, ArrayList<Double> accelerationList, double period) {
		this.positionList = positionList;
		this.velocityList = velocityList;
		this.accelerationList = accelerationList;
		this.period = period;
	}
	
	@Override
	public double getGoalVelocity(double time) {
		if(time > getTotalTimeOfTrajectory()) {
			return getEndVelocity();
		}
		return velocityList.get((int) (time/period));
	}

	@Override
	public double getGoalPosition(double time) {
		if(time > getTotalTimeOfTrajectory()) {
			return getEndPosition();
		}
		return positionList.get((int) (time/period));
	}

	@Override
	public double getGoalAccel(double time) {
		if(time > getTotalTimeOfTrajectory()) {
			return getEndAcceleration();
		}
		return accelerationList.get((int) (time/period));
	}

	@Override
	public double getMaxUsedVelocity() {
		double maxVelocity = 0;
		for(double d : velocityList) {
			if(d > maxVelocity) {
				maxVelocity = d;
			}
		}
		return maxVelocity;
	}

	@Override
	public double getMaxUsedAccel() {
		double maxAccel = 0;
		for(double d : accelerationList) {
			if(d > maxAccel) {
				maxAccel = d;
			}
		}
		return maxAccel;
	}
	
	public double getTotalTimeOfTrajectory() {
		return Math.min(positionList.size() - 1, Math.min(velocityList.size() - 1, accelerationList.size() - 1))* period;
	}
	
	public double getEndAcceleration() {
		return accelerationList.get(accelerationList.size() - 1);
	}
	
	public double getEndVelocity() {
		return velocityList.get(velocityList.size() - 1);
	}

	@Override
	public double getEndPosition() {
		return positionList.get(positionList.size() - 1);
	}

	@Override
	public double getGoalHeading(double time) {
		return 0;
	}

	@Override
	public ArrayList<Double> loadPosPoints(double period) {
		ArrayList<Double> posPoints = new ArrayList<Double>();
		for (int time = 0; time < getTotalTimeOfTrajectory(); time += period) {
			posPoints.add(getGoalPosition(time));
		}
		return posPoints;
	}

	@Override
	public ArrayList<Double> loadVelPoints(double period) {
		ArrayList<Double> velPoints = new ArrayList<Double>();
		for (int time = 0; time < getTotalTimeOfTrajectory(); time += period) {
			velPoints.add(getGoalVelocity(time));
		}
		return velPoints;
	}

	@Override
	public ArrayList<Double> loadAccelPoints(double period) {
		ArrayList<Double> accelPoints = new ArrayList<Double>();
		for (int time = 0; time < getTotalTimeOfTrajectory(); time += period) {
			accelPoints.add(getGoalVelocity(time));
		}
		return accelPoints;
	}

}
