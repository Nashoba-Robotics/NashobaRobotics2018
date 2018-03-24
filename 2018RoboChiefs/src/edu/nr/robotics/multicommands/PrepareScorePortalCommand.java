package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.intakeElevator.IntakeDeployCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareScorePortalCommand extends CommandGroup {

	public PrepareScorePortalCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				addParallel(new IntakeDeployCommand());
				
				addParallel(new IntakeRollersIntakeCommand());
			}
		});
	}
}
