package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomDropCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorBottomCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorFoldCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorHandlerCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareScoreElevatorBottomCommand extends CommandGroup {

	public PrepareScoreElevatorBottomCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				addParallel(new ElevatorBottomDropCommand());
				
				addParallel(new IntakeElevatorBottomCommand());
				
			}
		});
		
		addSequential(new IntakeRollersIntakeCommand());
		
		addSequential(new IntakeElevatorHandlerCommand());
		
		addSequential(new CubeFeedIntakeRollersToElevatorCommand());
		
		addSequential(new FoldIntakeMultiCommand());
	}
}
