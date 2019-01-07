package edu.nr.lib.motionprofiling;

import java.util.ArrayList;

public class OneDimensionalTrajectorySimple implements OneDimensionalTrajectory {
	
	double maxUsedVelocity;
	double maxUsedAccel;
	
	double totalTime;
	double timeAccelPlus;
	double timeAccelMinus;
	double timeAtCruise;
	
	double endPosition;
	double startPosition;
	
	ArrayList<Double> posPoints;
	ArrayList<Double> velPoints;
	ArrayList<Double> accelPoints;
	
	boolean triangleShaped;
		
	public OneDimensionalTrajectorySimple(double goalPositionDelta, double maxUsedVelocity, double maxUsedAccel) {
		
		posPoints = new ArrayList<Double>();
		velPoints = new ArrayList<Double>();
		accelPoints = new ArrayList<Double>();
		
		this.endPosition = goalPositionDelta;
		this.startPosition = 0;
		if(goalPositionDelta < 0) {
			maxUsedVelocity *= -1;
			maxUsedAccel *= -1; 
		}
		
		this.maxUsedVelocity = maxUsedVelocity;
		this.maxUsedAccel = maxUsedAccel; 
		
		timeAccelPlus = timeAccelMinus = maxUsedVelocity / maxUsedAccel;
		if(Math.abs(0.5 * (timeAccelPlus + timeAccelMinus) * maxUsedVelocity) >= Math.abs(goalPositionDelta)) {
			triangleShaped = true;
			timeAtCruise = 0;
			timeAccelPlus = timeAccelMinus = Math.sqrt(goalPositionDelta / maxUsedAccel);
		} else {
			triangleShaped = false;
			double cruiseDistance = goalPositionDelta - 0.5 * timeAccelPlus * maxUsedVelocity - 0.5 * timeAccelMinus * maxUsedVelocity;
			timeAtCruise = cruiseDistance / maxUsedVelocity;
		}
		
		//System.out.println("Triangle shaped: " + triangleShaped);
		
		totalTime = timeAccelMinus + timeAtCruise + timeAccelMinus;
	}

	public double getGoalVelocity(double time) {
		
		if(triangleShaped) {
			if(time <= 0) return 0;
			if(time <= timeAccelPlus) return time * maxUsedAccel;
			double timeSlowingDownSoFar = time - timeAccelPlus;
			if(time <= totalTime) return maxUsedVelocity + timeSlowingDownSoFar * -maxUsedAccel;
			return 0;
		} else {
			if(time <= 0) return 0;
			if(time <= timeAccelPlus) return time * maxUsedAccel;
			if(time <= timeAccelPlus + timeAtCruise) return maxUsedVelocity;
			double timeSlowingDownSoFar = time - (timeAccelPlus + timeAtCruise);
			if(time <= totalTime) return maxUsedVelocity + timeSlowingDownSoFar * -maxUsedAccel;
			return 0;
		}
	}
	
	public double getGoalPosition(double time) {	
		if(time > totalTime) {
			return endPosition;
		}
		if(triangleShaped) {
			if(time <= timeAccelPlus) {
				//We're on the positive slope of the trapezoid
				return 0.5 * time * time * maxUsedAccel + startPosition;
			}
			
			double speedUpDistance =  0.5 * timeAccelPlus * maxUsedVelocity;
			
			if(time <= totalTime) {
				//We're on the negative slope of the trapezoid
				double timeSlowingDownSoFar = time - (timeAccelPlus + timeAtCruise);
				return speedUpDistance 
						+ maxUsedVelocity * timeSlowingDownSoFar 
						- 0.5 * maxUsedAccel * timeSlowingDownSoFar * timeSlowingDownSoFar + startPosition;
			}
					
			return endPosition;
		} else  {
			if(time <= timeAccelPlus) {
				//We're on the positive slope of the trapezoid
				return 0.5 * time * time * maxUsedAccel + startPosition;
			}
			
			double speedUpDistance =  0.5 * timeAccelPlus * maxUsedVelocity;
			
			if(time <= timeAccelPlus + timeAtCruise && timeAtCruise > 0) {
				//We're on the top part of the trapezoid
				double timeAtFullSoFar = time - timeAccelPlus;
				return speedUpDistance + timeAtFullSoFar * maxUsedVelocity + startPosition;
			}
			
			double fullSpeedDistance = maxUsedVelocity * timeAtCruise;
			
			if(time <= totalTime) {
				//We're on the negative slope of the trapezoid
				double timeSlowingDownSoFar = time - (timeAccelPlus + timeAtCruise);
				return speedUpDistance + fullSpeedDistance 
						+ maxUsedVelocity * timeSlowingDownSoFar 
						- 0.5 * maxUsedAccel * timeSlowingDownSoFar * timeSlowingDownSoFar + startPosition;
			}
					
			return endPosition;
		}
	}

	public double getGoalAccel(double time) {
		if(time < 0) return 0;
		if(time < timeAccelPlus) return maxUsedAccel;
		if(time < timeAccelPlus + timeAtCruise && !triangleShaped) return 0;
		if(time < totalTime) return -maxUsedAccel;
		return 0;
	}
	
	@Override
	public double getMaxUsedVelocity() {
		return maxUsedVelocity;
	}

	@Override
	public double getMaxUsedAccel() {
		return maxUsedAccel;
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
	public ArrayList<Double> loadPosPoints(double period) {
		posPoints.clear();
		for (int time = 0; time < Math.round(totalTime * 100.0); time += period) {
			posPoints.add(getGoalPosition(time / 100.0));
		}
		return posPoints;
	}
	
	@Override
	public ArrayList<Double> loadVelPoints(double period) {
		velPoints.clear();
		for (int time = 0; time < Math.round(totalTime * 100.0); time += period) {
			velPoints.add(getGoalVelocity(time / 100.0));
		}
		return velPoints;
	}
	
	@Override
	public ArrayList<Double> loadAccelPoints(double period) {
		accelPoints.clear();
		for (int time = 0; time < Math.round(totalTime * 100.0); time += period) {
			accelPoints.add(getGoalAccel(time / 100.0));
		}
		return accelPoints;
	}

}