package edu.nr.robotics.subsystems.intakeRollers;

import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class IntakeRollersReverseCommand extends CommandGroup {

	public IntakeRollersReverseCommand(double percent) {
	
		addParallel(new IntakeRollersVelocityCommand(-percent, -percent));
		
		addSequential(new WaitCommand(IntakeRollers.SCORE_TIME.get(Time.Unit.SECOND)));
		
		addSequential(new IntakeRollersStopCommand());		
	}
}
