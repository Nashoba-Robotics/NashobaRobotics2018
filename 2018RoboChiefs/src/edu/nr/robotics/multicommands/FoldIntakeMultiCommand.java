package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorFoldCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersStopCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class FoldIntakeMultiCommand extends CommandGroup {

	public FoldIntakeMultiCommand() {
		
		addSequential(new IntakeRollersStopCommand());
		addSequential(new IntakeElevatorFoldCommand());
	}
}
