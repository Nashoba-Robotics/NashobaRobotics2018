package edu.nr.robotics.multicommands;

import edu.nr.lib.units.Time;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandler;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandlerStopCommand;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandlerVelocityCommand;
import edu.nr.robotics.subsystems.drive.StrafeToPortalCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ScorePortalCommand extends CommandGroup {

	public ScorePortalCommand(Direction direction) {
		
		addSequential(new StrafeToPortalCommand(direction));
		//addParallel(new CubeHandlerVelocityCommand(CubeHandler.VEL_PERCENT_CUBE_HANDLER));
		//addSequential(new WaitCommand(CubeHandler.SCORE_TIME.get(Time.Unit.SECOND)));
		//addSequential(new CubeHandlerStopCommand());
		
	}
}
