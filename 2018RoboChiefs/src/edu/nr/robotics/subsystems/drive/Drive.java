package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.units.Distance;

public class Drive {

	private static Drive singleton;
	
	public static final double WHEEL_DIAMETER_INCHES = 0; //TODO: Find real drive wheel diameter
	public static final Distance WHEEL_BASE = new Distance(0, Distance.Unit.INCH); //TODO: Find real drive wheel base
}
