package edu.nr.lib.commandbased;

import edu.nr.robotics.auton.AutoChoosers;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.AutoChoosers.StartPos;
import edu.nr.robotics.auton.AutoChoosers.Switch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StartAutoCommand extends NRCommand {
	
	public StartAutoCommand() {
		
	}
	
	@Override
	public void onStart() {
		
		AutoChoosers.autoStartPosChooser.addDefault("Left", StartPos.left);
		AutoChoosers.autoStartPosChooser.addObject("Middle", StartPos.middle);
		AutoChoosers.autoStartPosChooser.addObject("Right", StartPos.right);
		AutoChoosers.autoStartPosChooser.addObject("Far Right", StartPos.farRight);
		
		AutoChoosers.autoSwitchChooser.addDefault("Left Only", Switch.leftOnly);
		AutoChoosers.autoSwitchChooser.addObject("RightOnly", Switch.rightOnly);
		AutoChoosers.autoSwitchChooser.addObject("None", Switch.none);
		AutoChoosers.autoSwitchChooser.addObject("both", Switch.both);
		
		AutoChoosers.autoScaleChooser.addDefault("Scale", Scale.yes);
		AutoChoosers.autoScaleChooser.addObject("No Scale", Scale.no);
		
		AutoChoosers.StartPos selectedStartPos = AutoChoosers.autoStartPosChooser.getSelected();
		AutoChoosers.Switch selectedSwitch = AutoChoosers.autoSwitchChooser.getSelected();
		AutoChoosers.Scale selectedScale = AutoChoosers.autoScaleChooser.getSelected();
		
		
	}
	
}
