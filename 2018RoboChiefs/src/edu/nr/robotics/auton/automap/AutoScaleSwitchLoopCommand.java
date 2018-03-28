package edu.nr.robotics.auton.automap;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.Robot;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.AutoChoosers.AllianceBlocks;
import edu.nr.robotics.auton.AutoChoosers.Switch;
import edu.nr.robotics.auton.autoroutes.ScaleToBlockProfilingCommand;
import edu.nr.robotics.multicommands.PrepareScoreSwitchAutoCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class AutoScaleSwitchLoopCommand extends CommandGroup {
	
	public AutoScaleSwitchLoopCommand() {
		
		addSequential(new ConditionalCommand(new ScaleToBlockProfilingCommand(1)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new ScaleToBlockProfilingCommand(6)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.right;
			}

		});
		
		addSequential(new PrepareScoreSwitchAutoCommand());
		
				
	}

}
