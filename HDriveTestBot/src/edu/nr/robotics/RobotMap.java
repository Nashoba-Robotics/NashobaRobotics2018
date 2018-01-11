package edu.nr.robotics;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public static final int FRONT_RIGHT_TALON_ID = 0; 	//master
	public static final int FRONT_LEFT_TALON_ID = 0; 	//master
	public static final int BACK_RIGHT_TALON_ID = 0; 	//follower
	public static final int BACK_LEFT_TALON_ID = 0; 	//follower
	public static final int H_TALON_ID = 0; 			//master
	public static final int H_TALON_FOLLOW_ID = 0;		//follower
}
