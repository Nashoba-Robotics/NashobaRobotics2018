package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorPositionCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareScorePortalCommand extends CommandGroup {

	public PrepareScorePortalCommand() {
		
		addSequential(new IntakeElevatorPositionCommand(IntakeElevator.PORTAL_HEIGHT));

	}
}
