package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeDeployCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareCubeIntakeCommand extends CommandGroup {

	public PrepareCubeIntakeCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				addParallel(new IntakeDeployCommand());
				addParallel(new ElevatorBottomCommand());
			}
		});
	}
}
