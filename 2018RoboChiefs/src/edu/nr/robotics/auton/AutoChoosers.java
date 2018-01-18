package edu.nr.robotics.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoChoosers {
	
	public static SendableChooser<AutoChoosers.StartPos> autoStartPosChooser = new SendableChooser<>();
	public static SendableChooser<AutoChoosers.Switch> autoSwitchChooser = new SendableChooser<>();
	public static SendableChooser<AutoChoosers.Scale> autoScaleChooser = new SendableChooser<>();
	public static ProfilingMethod chosen = ProfilingMethod.basic;
	
	public enum StartPos {
		left,
		middle,
		right,
		farRight;
	}
	
	public enum Switch {
		leftOnly,
		rightOnly,
		none,
		both;
	}
	
	public enum Scale {
		yes,
		no;
	}
	
	public enum ProfilingMethod {
		basic,
		oneD;
	}
}
