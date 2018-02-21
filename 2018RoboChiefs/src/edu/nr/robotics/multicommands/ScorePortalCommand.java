package edu.nr.robotics.multicommands;

import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersStopCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersVelocityCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ScorePortalCommand extends CommandGroup {

	public ScorePortalCommand() {

		addParallel(new IntakeRollersVelocityCommand(-IntakeRollers.VEL_PERCENT_HIGH_INTAKE_ROLLERS, -IntakeRollers.VEL_PERCENT_HIGH_INTAKE_ROLLERS));
		
		addSequential(new WaitCommand(IntakeRollers.SCORE_TIME.get(Time.Unit.SECOND)));
		
		addSequential(new IntakeRollersStopCommand());		
	}
}
