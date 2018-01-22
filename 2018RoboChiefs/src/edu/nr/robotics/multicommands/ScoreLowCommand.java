package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.cubeHandler.CubeHandler;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandlerVelocityCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ScoreLowCommand extends CommandGroup {
	
	public ScoreLowCommand() {
		addSequential(new ElevatorPositionCommand(Elevator.SCORE_LOW_HEIGHT_ELEVATOR));
		addSequential(new CubeHandlerVelocityCommand(CubeHandler.VEL_PERCENT_CUBE_HANDLER));
		//TODO: Make a wait command or use a sensor in parallel to stop the cube handler
		
	}

}
