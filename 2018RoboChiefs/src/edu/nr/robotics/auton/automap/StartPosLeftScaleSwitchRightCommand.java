package edu.nr.robotics.auton.automap;

import edu.nr.robotics.auton.autoroutes.StartPosLeftToScaleRightProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosLeftScaleSwitchRightCommand extends CommandGroup {

	public StartPosLeftScaleSwitchRightCommand() {
		
		addSequential(new StartPosLeftToScaleRightProfilingCommand());
		
		addSequential(new AutoScaleSwitchLoopCommand());
		
	}
}
