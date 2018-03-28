package edu.nr.robotics.auton.automap;

import edu.nr.robotics.auton.autoroutes.StartPosLeftToScaleLeftProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosRightScaleSwitchLeftCommand extends CommandGroup {

	public StartPosRightScaleSwitchLeftCommand() {
		
		addSequential(new StartPosLeftToScaleLeftProfilingCommand());
		
		
		
	}
	
}
