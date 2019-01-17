package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorFoldCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersStopCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;

public class FoldIntakeMultiCommand extends CommandGroup {

	public FoldIntakeMultiCommand() {
		
		addSequential(new IntakeRollersStopCommand());
		addSequential(new PrintCommand("Don't fold the intake. Fold the team."));
		addSequential(new IntakeElevatorFoldCommand());
		
	}
}
