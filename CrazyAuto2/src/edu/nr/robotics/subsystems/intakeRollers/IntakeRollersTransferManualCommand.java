package edu.nr.robotics.subsystems.intakeRollers;

import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class IntakeRollersTransferManualCommand extends CommandGroup {

	public IntakeRollersTransferManualCommand() {
		
		addSequential(new IntakeRollersVelocityCommand(IntakeRollers.VEL_PERCENT_TRANSFER_INTAKE_ROLLERS, IntakeRollers.VEL_PERCENT_TRANSFER_INTAKE_ROLLERS));
		addSequential(new WaitCommand(IntakeRollers.SCORE_TIME.get(Time.Unit.SECOND)));
		addSequential(new IntakeRollersStopCommand());
	}
}
