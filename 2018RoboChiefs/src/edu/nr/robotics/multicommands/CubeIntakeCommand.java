package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class CubeIntakeCommand extends CommandGroup {
	
	public CubeIntakeCommand() {
		
		addSequential(new IntakeElevatorPositionCommand(IntakeElevator.INTAKE_HEIGHT));
		
		addSequential(new IntakeRollersIntakeCommand());
	}

}
