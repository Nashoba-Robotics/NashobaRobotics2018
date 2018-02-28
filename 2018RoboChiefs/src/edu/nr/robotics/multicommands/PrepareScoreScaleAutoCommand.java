package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomDropCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorBottomCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorProfileCommandGroup;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareScoreScaleAutoCommand extends CommandGroup {

	public PrepareScoreScaleAutoCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				addParallel(new ElevatorBottomDropCommand());
				addParallel(new IntakeElevatorProfileCommandGroup(IntakeElevator.HANDLER_HEIGHT,
						IntakeElevator.PROFILE_VEL_PERCENT_INTAKE_ELEVATOR,
						IntakeElevator.PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR));
				
			}
		});
		
		addSequential(new CubeFeedIntakeRollersToElevatorCommand());
		
		addSequential(new ElevatorProfileCommandGroup(Elevator.SCALE_HEIGHT_ELEVATOR,
				Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
	}
}
