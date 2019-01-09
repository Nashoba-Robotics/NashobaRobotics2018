package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class ElevatorProfileDeltaCommandGroup extends CommandGroup {

	public ElevatorProfileDeltaCommandGroup(Distance delta, double maxVelPercent, double maxAccelPercent) {
		
		addSequential(new ElevatorProfileDeltaBasicCommand(delta, maxVelPercent, maxAccelPercent));
	
		/*addSequential(new ConditionalCommand(new ElevatorPositionSmartDashboardCommand()) {
			
			@Override
			protected boolean condition() {
				return (Elevator.getInstance().getPosition().sub(delta.add(Elevator.getInstance().getPosition()))).abs().greaterThan(Elevator.PROFILE_END_POS_THRESHOLD_ELEVATOR);
			}
		});*/
	}
}
