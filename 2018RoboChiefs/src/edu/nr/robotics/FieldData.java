package edu.nr.robotics;

import edu.wpi.first.wpilibj.DriverStation;

public class FieldData {
	
	//TODO: print this to SmartDashboard
	public enum Direction {
		left, right;
	}
	
	public enum Color {
		red, blue;
	}
	
	public static Direction nearSwitch;
	public static Direction farSwitch;
	public static Direction scale;
	
	public static void getFieldData() { 
		String gsm = DriverStation.getInstance().getGameSpecificMessage();
		
		if (gsm.charAt(0) == 'l' || gsm.charAt(0) == 'L'){
			nearSwitch = Direction.left;
		} else if (gsm.charAt(0) == 'r' || gsm.charAt(0) == 'R'){
			nearSwitch = Direction.right;
		}
		
		if (gsm.charAt(1) == 'l' || gsm.charAt(1) == 'L'){
			scale = Direction.left;
		} else if (gsm.charAt(1) == 'r' || gsm.charAt(1) == 'R'){
			scale = Direction.right;
		}
		
		if (gsm.charAt(2) == 'l' || gsm.charAt(2) == 'L'){
			farSwitch = Direction.left;
		} else if (gsm.charAt(0) == 'r' || gsm.charAt(0) == 'R'){
			farSwitch = Direction.right;
		}
	}
}
