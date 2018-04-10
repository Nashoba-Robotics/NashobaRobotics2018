package edu.nr.robotics.auton;

import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoChoosers {
	
	public static SendableChooser<AutoChoosers.StartPos> autoStartPosChooser = new SendableChooser<>();
	public static SendableChooser<AutoChoosers.Switch> autoSwitchChooser = new SendableChooser<>();
	public static SendableChooser<AutoChoosers.Scale> autoScaleChooser = new SendableChooser<>();
	public static SendableChooser<AutoChoosers.AllianceBlocks> allianceBlockChooser = new SendableChooser<>();
	public static SendableChooser<AutoChoosers.ScaleSwitch> autoScaleSwitchChooser = new SendableChooser<>();
	
	public static ProfilingMethod chosen = ProfilingMethod.oneD;
	public static boolean strafe = false;
	
	public enum StartPos {
		left,
		middle,
		right;
	}
	
	public enum Switch {
		leftOnly,
		rightOnly,
		other,
		both;
	}
	
	public enum Scale {
		both,
		leftonly,
		rightonly,
		none;
	}
	
	public enum AllianceBlocks {
		none,
		block1,
		block6,
		both;
	}
	
	public enum ProfilingMethod {
		basic,
		oneD;
	}
	
	public enum ScaleSwitch {
		scaleSwitchYes,
		scaleSwitchNo;
	}
	
	
}
