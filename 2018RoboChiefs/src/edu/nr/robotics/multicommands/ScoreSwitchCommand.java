package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterStopCommand;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterVelocityCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorPositionCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ScoreSwitchCommand extends CommandGroup {
	
	public ScoreSwitchCommand() {

		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				addParallel(new IntakeElevatorPositionCommand(IntakeElevator.HANDLER_HEIGHT));
				addParallel(new ElevatorPositionCommand(Elevator.BOTTOM_HEIGHT_ELEVATOR));
			}
			
		});
		
		addSequential(new CubeFeedIntakeRollersToElevatorCommand());
		addSequential(new ElevatorPositionCommand(Elevator.SCORE_LOW_HEIGHT_ELEVATOR));
		addParallel(new ElevatorShooterVelocityCommand(ElevatorShooter.VEL_PERCENT_SWITCH_ELEVATOR_SHOOTER));
		addSequential(new WaitCommand(ElevatorShooter.SHOOT_TIME.get(Time.Unit.SECOND)));
		addSequential(new ElevatorShooterStopCommand());
		
	}

}
