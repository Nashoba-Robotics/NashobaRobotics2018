package edu.nr.lib.gyro;

import edu.nr.lib.units.Angle;

public abstract class Gyro {

	public enum ChosenGyro {
		NavX, Pigeon
	}
	
	//This is the one line where gyro type can be chosen
	public static ChosenGyro chosenGyro = ChosenGyro.NavX;
	
	public abstract Angle getYaw();
	public abstract void reset();
}
