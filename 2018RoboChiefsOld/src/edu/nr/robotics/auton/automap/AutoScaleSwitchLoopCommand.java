package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.autoroutes.BlockToScaleProfilingCommand;
import edu.nr.robotics.auton.autoroutes.Cube1ToSwitchProfilingCommand;
import edu.nr.robotics.auton.autoroutes.ScaleToBlockProfilingCommand;
import edu.nr.robotics.auton.autoroutes.SwitchToCube2ProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class AutoScaleSwitchLoopCommand extends CommandGroup {
	
	public AutoScaleSwitchLoopCommand() {
		
		addSequential(new ConditionalCommand(new ScaleToBlockProfilingCommand(1), new ScaleToBlockProfilingCommand(6)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.left;
			}

		});
		
		addSequential(new Cube1ToSwitchProfilingCommand());
		
		addSequential(new SwitchToCube2ProfilingCommand());
		
		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(2), new BlockToScaleProfilingCommand(5)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.left;
			}

		});
		
		addSequential(new ConditionalCommand(new ScaleToBlockProfilingCommand(3), new ScaleToBlockProfilingCommand(4)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(3), new BlockToScaleProfilingCommand(4)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.left;
			}

		});
				
	}

}
