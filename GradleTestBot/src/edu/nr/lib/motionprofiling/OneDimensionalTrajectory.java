package edu.nr.lib.motionprofiling;

import java.util.ArrayList;

public interface OneDimensionalTrajectory {

	public double getGoalVelocity(double time);
	
	public double getGoalPosition(double time);

	public double getGoalAccel(double time);
	
	public double getGoalHeading(double time);
	
	public double getMaxUsedVelocity();

	public double getMaxUsedAccel();

	public double getEndPosition();

	public ArrayList<Double> loadPosPoints(double period);

	public ArrayList<Double> loadVelPoints(double period);

	public ArrayList<Double> loadAccelPoints(double period);

}
