package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class IntakeDeployCommand extends CommandGroup {
	
	public IntakeDeployCommand() {
	
		addParallel(new IntakeElevatorVelocityCommand(-IntakeElevator.DEPLOY_PERCENT_INTAKE_ELEVATOR));
		addSequential(new WaitCommand(IntakeElevator.INTAKE_FLIP_OUT_TIME.get(Time.Unit.SECOND)));
		addSequential(new IntakeElevatorBottomCommand());
		
	}
	
	
}
