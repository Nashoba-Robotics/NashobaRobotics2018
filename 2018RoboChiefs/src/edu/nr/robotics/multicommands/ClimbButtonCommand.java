package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.climber.ClimbBasicPercentCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorSetToZeroCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbButtonCommand extends CommandGroup {
	
	public ClimbButtonCommand() {
		
		addParallel(new ElevatorSetToZeroCommand());
		addSequential(new ClimbBasicPercentCommand());
		
	}
	
}