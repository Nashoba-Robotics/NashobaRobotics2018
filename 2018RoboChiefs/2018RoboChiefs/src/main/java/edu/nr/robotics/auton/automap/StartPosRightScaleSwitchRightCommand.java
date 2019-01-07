package edu.nr.robotics.auton.automap;

import edu.nr.robotics.auton.autoroutes.StartPosRightToScaleRightProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosRightScaleSwitchRightCommand extends CommandGroup {
	
	public StartPosRightScaleSwitchRightCommand() {
		
		addSequential(new StartPosRightToScaleRightProfilingCommand());
		
		addSequential(new AutoScaleSwitchLoopCommand());
		
	}

}
