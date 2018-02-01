package edu.nr.robotics.auton.automap;

import edu.nr.lib.commandbased.DoNothingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoSwitchLoopCommand extends CommandGroup {

	public AutoSwitchLoopCommand() {
		
		addSequential(new DoNothingCommand());
		
	}
}
