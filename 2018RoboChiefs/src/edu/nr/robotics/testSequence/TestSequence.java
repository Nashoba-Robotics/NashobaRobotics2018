package edu.nr.robotics.testSequence;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomDropCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterStopCommand;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterVelocityCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorBottomCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersStopCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersVelocityCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class TestSequence extends CommandGroup {

	private final double STARTUP_SEQUENCE_WAIT = 3; //In seconds
	
	public TestSequence() {
		
		addParallel(new SystemTestSequenceCommand());
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				//Test Elevator
				addParallel(new ElevatorProfileCommandGroup(new Distance(6, Distance.Unit.INCH), Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
				
				//Test ElevatorShooter
				addParallel(new ElevatorShooterVelocityCommand(ElevatorShooter.VEL_PERCENT_HIGH_ELEVATOR_SHOOTER));
				
				//Test IntakeElevator
				addParallel(new IntakeElevatorBottomCommand());
				
				//Test IntakeRollers
				addParallel(new IntakeRollersVelocityCommand(IntakeRollers.VEL_PERCENT_HIGH_INTAKE_ROLLERS, IntakeRollers.VEL_PERCENT_LOW_INTAKE_ROLLERS));
				
				addSequential(new WaitCommand(STARTUP_SEQUENCE_WAIT));
				
				//Test Elevator
				addParallel(new ElevatorBottomDropCommand());
				
				//Test Elevator Shooter
				addParallel(new ElevatorShooterStopCommand());
				
				//Test Intake Elevator
				addParallel(new IntakeElevatorPositionCommand(IntakeElevator.FOLDED_HEIGHT));
				
				//Test Intake Rollers
				addParallel(new IntakeRollersStopCommand());
				
				addSequential(new WaitCommand(STARTUP_SEQUENCE_WAIT));				
			}
		});
	}
}
