package edu.nr.lib.gyro;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;

/**
 * 
 *  This is an extendible class that also allows us to choose which gyro we are using
 *  
 */

public abstract class Gyro {

	public enum ChosenGyro {
		NavX, Pigeon
	}
	
	//This is the one line where gyro type can be chosen
	public static ChosenGyro chosenGyro = ChosenGyro.Pigeon;
	
	public abstract Angle getYaw();
	public abstract void reset();
	public abstract Distance getYError();
}
