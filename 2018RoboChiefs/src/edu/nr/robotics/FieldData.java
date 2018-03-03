package edu.nr.robotics;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldData {
	
	//TODO: print this to SmartDashboard
	public enum Direction {
		left, right;
	}
	
	public Direction nearSwitch;
	public Direction farSwitch;
	public Direction scale;
	public Alliance alliance;
	
	public static FieldData singleton;
	
	public static FieldData getInstance() {
		if (singleton == null)
			init();
		return singleton;
	}
	
	public synchronized static void init() {
		if (singleton == null) {
			singleton = new FieldData();
		}
	}
	
	public void getFieldData() { 
		String gsm = DriverStation.getInstance().getGameSpecificMessage();
		
		System.out.println(gsm.length());
		
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
		
		alliance = DriverStation.getInstance().getAlliance();
	}
}
