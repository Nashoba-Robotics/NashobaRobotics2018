package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandlerStopCommand;
import edu.nr.robotics.subsystems.drive.StrafeToPortalCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersStopCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PrepareScorePortalCommand extends CommandGroup {

	public PrepareScorePortalCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				addParallel(new IntakeElevatorPositionCommand(IntakeElevator.HANDLER_HEIGHT));
				addParallel(new ElevatorPositionCommand(Elevator.SWITCH_HEIGHT_ELEVATOR));
			}
			
		});
		
		addSequential(new CubeFeedIntakeRollersToCubeHandlerCommand());
	}
}
