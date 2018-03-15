package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class ElevatorProfileCommandGroup extends CommandGroup {

	public ElevatorProfileCommandGroup(Distance position, double maxVelPercent, double maxAccelPercent) {
		
		addSequential(new ElevatorProfileBasicCommand(position, maxVelPercent, maxAccelPercent));
		
		addSequential(new ConditionalCommand(new ElevatorPositionCommand(position)) {
			
			@Override
			protected boolean condition() {
				return (Elevator.getInstance().getPosition().sub(position)).abs().greaterThan(Elevator.PROFILE_END_POS_THRESHOLD_ELEVATOR);
			}
		});
		
	}
}
