package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomDropCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorHandlerCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareScoreSwitchCommand extends CommandGroup {
	
	public PrepareScoreSwitchCommand() {

		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				addParallel(new IntakeElevatorHandlerCommand());
				addParallel(new ElevatorBottomDropCommand());
			}
			
		});
		
		addSequential(new CubeFeedIntakeRollersToElevatorCommand());
		addSequential(new ElevatorBottomDropCommand());
	
	}

}
