package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ElevatorProfileDeltaCommandGroup extends CommandGroup {

	public ElevatorProfileDeltaCommandGroup(Distance delta, double maxVelPercent, double maxAccelPercent) {
		
		addSequential(new ElevatorProfileBasicCommand(delta.add(Elevator.getInstance().getPosition()), maxVelPercent, maxAccelPercent));
	}
}
