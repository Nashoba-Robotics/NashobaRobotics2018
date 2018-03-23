package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.intakeElevator.IntakeDeployCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class PrepareScoreElevatorBottomCommand extends CommandGroup {

	/**
	 * Time waited between sensing of cube in intake and raising intake arms
	 */
	public final Time INTAKE_CUBE_WAIT_TIME = new Time(0.25, Time.Unit.SECOND);
	
	public PrepareScoreElevatorBottomCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				addParallel(new ElevatorBottomCommand());
				
				addParallel(new IntakeDeployCommand());
				
			}
		});
		
		addSequential(new IntakeRollersIntakeCommand());
		
		addSequential(new WaitCommand(INTAKE_CUBE_WAIT_TIME.get(Time.Unit.SECOND)));
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				addParallel(new IntakeElevatorProfileCommandGroup(IntakeElevator.HANDLER_HEIGHT, IntakeElevator.PROFILE_VEL_PERCENT_INTAKE_ELEVATOR, IntakeElevator.PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR));
				
				addParallel(new ElevatorProfileCommandGroup(Elevator.TRANSFER_HEIGHT_ELEVATOR, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
			}
		});
		
		addSequential(new CubeFeedIntakeRollersToElevatorCommandManual());
		
	}
}
