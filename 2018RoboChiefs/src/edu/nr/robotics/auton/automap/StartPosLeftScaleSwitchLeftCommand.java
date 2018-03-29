package edu.nr.robotics.auton.automap;

import edu.nr.robotics.auton.autoroutes.StartPosLeftToScaleLeftProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosLeftScaleSwitchLeftCommand extends CommandGroup {

	public StartPosLeftScaleSwitchLeftCommand() {

		addSequential(new StartPosLeftToScaleLeftProfilingCommand());
		
		addSequential(new AutoScaleSwitchLoopCommand());
		
	}
}
