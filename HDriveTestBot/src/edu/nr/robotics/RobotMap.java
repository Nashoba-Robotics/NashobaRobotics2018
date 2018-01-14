package edu.nr.robotics;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public static final int RIGHT_TALON_ID = 5; 		//master
	public static final int LEFT_TALON_ID = 2; 			//master
	public static final int RIGHT_FOLLOW_TALON_ID = 1; 	//follower
	public static final int LEFT_FOLLOW_TALON_ID = 4; 	//follower
	public static final int H_TALON_ID = 6; 			//master
	public static final int H_TALON_FOLLOW_ID = 3;		//follower
}
