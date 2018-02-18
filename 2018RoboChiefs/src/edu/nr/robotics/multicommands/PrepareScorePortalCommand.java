package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorPositionCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareScorePortalCommand extends CommandGroup {

	public PrepareScorePortalCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				addParallel(new IntakeElevatorPositionCommand(IntakeElevator.HANDLER_HEIGHT));
				addParallel(new ElevatorPositionCommand(Elevator.SWITCH_HEIGHT_ELEVATOR));
			}
			
		});
		
		addSequential(new IntakeElevatorPositionCommand(IntakeElevator.INTAKE_HEIGHT));//TODO: Where to put intake
	}
}
