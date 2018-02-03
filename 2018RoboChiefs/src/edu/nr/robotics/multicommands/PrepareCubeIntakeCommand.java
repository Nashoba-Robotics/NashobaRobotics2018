package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorPositionCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareCubeIntakeCommand extends CommandGroup {

	public PrepareCubeIntakeCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				addParallel(new IntakeElevatorPositionCommand(IntakeElevator.INTAKE_HEIGHT));
				addParallel(new ElevatorPositionCommand(Elevator.BOTTOM_HEIGHT_ELEVATOR));
			}
		});
	}
}
