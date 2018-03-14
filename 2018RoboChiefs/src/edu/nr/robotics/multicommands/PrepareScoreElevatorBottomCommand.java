package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomDropCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorBottomCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorFoldCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorHandlerCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class PrepareScoreElevatorBottomCommand extends CommandGroup {

	public PrepareScoreElevatorBottomCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				addParallel(new ElevatorBottomDropCommand());
				
				addParallel(new IntakeElevatorBottomCommand());
				
			}
		});
		
		addSequential(new IntakeRollersIntakeCommand());
		
		addSequential(new WaitCommand(0.25));
		
		addSequential(new IntakeElevatorProfileCommandGroup(IntakeElevator.HANDLER_HEIGHT, IntakeElevator.PROFILE_VEL_PERCENT_INTAKE_ELEVATOR, IntakeElevator.PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR));
		
		addSequential(new CubeFeedIntakeRollersToElevatorCommand());
		
	}
}