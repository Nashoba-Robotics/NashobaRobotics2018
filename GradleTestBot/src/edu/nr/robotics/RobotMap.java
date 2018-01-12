package edu.nr.robotics;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public static final int DRIVE_LEFT = 1;
	public static final int DRIVE_RIGHT = 2;
	public static final int DRIVE_RIGHT_FOLLOW = 4;
	public static final int DRIVE_LEFT_FOLLOW = 5;
	public static final int PIGEON_TALON = 3;
}
