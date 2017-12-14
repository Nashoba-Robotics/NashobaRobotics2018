package edu.nr.lib.gyro;

import edu.nr.lib.units.Angle;

public abstract class Gyro {

	public enum ChosenGyro {
		NavX, Pigeon
	}
	
	public static ChosenGyro chosenGyro = ChosenGyro.Pigeon;
	
	public abstract Angle getYaw();
	public abstract void reset();
}
