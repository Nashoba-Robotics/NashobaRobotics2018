package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbPrepareCommand extends CommandGroup {

	public ClimbPrepareCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				addParallel(new ElevatorProfileCommandGroup(Elevator.CLIMB_HEIGHT_ELEVATOR,
						Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
				
				addParallel(new FoldIntakeMultiCommand());
			}
		});		
	}
}
