package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorFoldCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorProfileCommandGroup;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareScoreSwitchAutoCommand extends CommandGroup {

	public PrepareScoreSwitchAutoCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				addParallel(new ElevatorBottomCommand());
				addParallel(new IntakeElevatorProfileCommandGroup(IntakeElevator.HANDLER_HEIGHT,
						IntakeElevator.PROFILE_VEL_PERCENT_INTAKE_ELEVATOR,
						IntakeElevator.PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR));
				
			}
		});
		
		addSequential(new CubeFeedIntakeRollersToElevatorCommand());
		addSequential(new IntakeElevatorFoldCommand());
	}
}
