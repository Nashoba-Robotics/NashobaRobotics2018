package edu.nr.robotics.multicommands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class DriveToCubeButtonCommand extends CommandGroup {

	public DriveToCubeButtonCommand(boolean advanced) {
			
		addSequential(new ConditionalCommand(new DriveToCubeCommandAdvanced(), new DriveToCubeCommandBasic()) {
			
			@Override
			protected boolean condition() {
				return advanced;
			}
		});
	}
}
