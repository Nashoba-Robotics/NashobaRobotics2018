package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterIntakeCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersTransferCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class CubeFeedIntakeRollersToElevatorCommand extends CommandGroup {
	
	public CubeFeedIntakeRollersToElevatorCommand() {
		
		addSequential(new IntakeElevatorPositionCommand(IntakeElevator.HANDLER_HEIGHT));
		
		addSequential(new AnonymousCommandGroup() {

			@Override
			public void commands() {
			
				addParallel(new ElevatorShooterIntakeCommand());
				
				addParallel(new IntakeRollersTransferCommand());
				
			}
			
		});
		
	}
	
}
