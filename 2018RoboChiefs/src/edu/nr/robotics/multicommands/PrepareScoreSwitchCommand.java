package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomDropCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorFoldCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorHandlerCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareScoreSwitchCommand extends CommandGroup {
	
	public PrepareScoreSwitchCommand() {

		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				addParallel(new IntakeElevatorHandlerCommand());
				addParallel(new ElevatorBottomDropCommand());
			}
			
		});
		
		addSequential(new CubeFeedIntakeRollersToElevatorCommand());
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				addParallel(new ElevatorProfileCommandGroup(Elevator.SWITCH_HEIGHT_ELEVATOR, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
				addParallel(new IntakeElevatorFoldCommand());
			}
		});
	}

}
