package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterIntakeCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersTransferCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersTransferManualCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class CubeFeedIntakeRollersToElevatorCommandManual extends CommandGroup {

	public CubeFeedIntakeRollersToElevatorCommandManual() {
		
		addSequential(new AnonymousCommandGroup() {

			@Override
			public void commands() {
			
				addParallel(new ElevatorShooterIntakeCommand());
				
				addParallel(new IntakeRollersTransferManualCommand());
				
			}
			
		});
	}
}
