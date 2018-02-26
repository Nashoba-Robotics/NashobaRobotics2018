package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.climber.ClimberPercentCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomDropCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbButtonCommand extends CommandGroup {
	
	public ClimbButtonCommand() {
		
		addParallel(new ElevatorBottomDropCommand());
		addSequential(new ClimberPercentCommand());
		
	}
	
}