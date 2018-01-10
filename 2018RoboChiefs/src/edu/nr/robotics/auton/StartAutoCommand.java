package edu.nr.robotics.auton;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.auton.AutoChoosers;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.AutoChoosers.StartPos;
import edu.nr.robotics.auton.AutoChoosers.Switch;
import edu.nr.robotics.auton.automap.StarPosMiddleSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosFarRightSwitchNoneCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchBothCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchNoneCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchBothCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchNoneCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchRightCommand;
import edu.nr.robotics.auton.automap.StartPosRightSwitchBothCommand;
import edu.nr.robotics.auton.automap.StartPosRightSwitchNoneCommand;
import edu.nr.robotics.auton.automap.StartPosRightSwitchRightCommand;

public class StartAutoCommand extends NRCommand {
	
	public AutoChoosers.StartPos selectedStartPos;
	public AutoChoosers.Switch selectedSwitch;
	public AutoChoosers.Scale selectedScale;
	
	public StartAutoCommand() {
		
	}
	
	@Override
	public void onStart() {
		sendableChoosers();
		
		if (selectedStartPos == AutoChoosers.StartPos.left) {
			if (selectedSwitch == AutoChoosers.Switch.none) {
				new StartPosLeftSwitchNoneCommand();
				
			} else if (selectedSwitch == AutoChoosers.Switch.leftOnly) {
				new StartPosLeftSwitchLeftCommand();
				
			} else if (selectedSwitch == AutoChoosers.Switch.both) {
				new StartPosLeftSwitchBothCommand();
				
			}
			
		} else if (selectedStartPos == AutoChoosers.StartPos.middle) {
			if (selectedSwitch == AutoChoosers.Switch.none) {
				new StartPosMiddleSwitchNoneCommand();
				
			} else if (selectedSwitch == AutoChoosers.Switch.leftOnly) {
				new StarPosMiddleSwitchLeftCommand();
				
			} else if (selectedSwitch == AutoChoosers.Switch.rightOnly){
				new StartPosMiddleSwitchRightCommand();
				
			} else if (selectedSwitch == AutoChoosers.Switch.both) {
				new StartPosMiddleSwitchBothCommand();
			}

		} else if (selectedStartPos == AutoChoosers.StartPos.right) {
			if (selectedSwitch == AutoChoosers.Switch.none) {
				new StartPosRightSwitchNoneCommand();
				
			} else if (selectedSwitch == AutoChoosers.Switch.rightOnly){
				new StartPosRightSwitchRightCommand();
				
			} else if (selectedSwitch == AutoChoosers.Switch.both) {
				new StartPosRightSwitchBothCommand();
			}
			
		} else if (selectedStartPos == AutoChoosers.StartPos.farRight) {
			if (selectedSwitch == AutoChoosers.Switch.none) {
				new StartPosFarRightSwitchNoneCommand();
				
			} else if (selectedSwitch == AutoChoosers.Switch.rightOnly){
				new StartPosRightSwitchRightCommand();
			}
		}
	}
	
	public void sendableChoosers(){
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
		
		selectedStartPos = AutoChoosers.autoStartPosChooser.getSelected();
		selectedSwitch = AutoChoosers.autoSwitchChooser.getSelected();
		selectedScale = AutoChoosers.autoScaleChooser.getSelected();
	}
	
}
