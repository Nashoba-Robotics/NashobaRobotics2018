package edu.nr.robotics.subsystems.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class ElevatorProfileSmartDashboardCommandGroup extends CommandGroup {

	public ElevatorProfileSmartDashboardCommandGroup() {
		
		addSequential(new ElevatorProfileBasicSmartDashboardCommand());

		addSequential(new ConditionalCommand(new ElevatorPositionSmartDashboardCommand()) {
			
			@Override
			protected boolean condition() {
				return (Elevator.getInstance().getPosition().sub(Elevator.profilePos)).abs().greaterThan(Elevator.PROFILE_END_POS_THRESHOLD_ELEVATOR);
			}
		});
	}
}
