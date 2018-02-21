package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class IntakeElevatorProfileCommandGroup extends CommandGroup {

	public IntakeElevatorProfileCommandGroup(Distance position, double maxVelPercent, double maxAccelPercent) {
		
		addSequential(new IntakeElevatorProfileBasicCommand(position, maxVelPercent, maxAccelPercent));
		addSequential(new IntakeElevatorPositionCommand(position));
	}
}
