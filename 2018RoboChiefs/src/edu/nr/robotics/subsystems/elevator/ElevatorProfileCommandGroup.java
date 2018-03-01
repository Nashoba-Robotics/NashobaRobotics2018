package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ElevatorProfileCommandGroup extends CommandGroup {

	public ElevatorProfileCommandGroup(Distance position, double maxVelPercent, double maxAccelPercent) {
		
		addSequential(new ElevatorProfileBasicCommand(position, maxVelPercent, maxAccelPercent));
		//addSequential(new ElevatorPositionCommand(position));
		
	}
}
