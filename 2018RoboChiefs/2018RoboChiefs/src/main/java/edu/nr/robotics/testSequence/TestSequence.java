package edu.nr.robotics.testSequence;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.multicommands.ClimberStopButtonCommand;
import edu.nr.robotics.subsystems.climber.Climber;
import edu.nr.robotics.subsystems.climber.ClimberPercentCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterStopCommand;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterVelocityCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeDeployCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorFoldCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersStopCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersVelocityCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class TestSequence extends CommandGroup {
	
	public TestSequence() {
		
		addParallel(new SystemTestSequenceCommand());
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				//Test IntakeRollers
				addSequential(new IntakeRollersVelocityCommand(IntakeRollers.VEL_PERCENT_HIGH_INTAKE_ROLLERS, IntakeRollers.VEL_PERCENT_LOW_INTAKE_ROLLERS));
				addSequential(new WaitCommand(0.5));
				addSequential(new IntakeRollersStopCommand());
				
				//Test IntakeElevator
				addSequential(new IntakeDeployCommand());
				//addSequential(new IntakeElevatorFoldCommand());
				
				//Test ElevatorShooter
				addSequential(new ElevatorShooterVelocityCommand(ElevatorShooter.VEL_PERCENT_HIGH_ELEVATOR_SHOOTER));
				addSequential(new WaitCommand(0.5));
				addSequential(new ElevatorShooterStopCommand());
				
				//Test Elevator
				addSequential(new ElevatorProfileCommandGroup(new Distance(12, Distance.Unit.INCH), Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
				addSequential(new ElevatorBottomCommand());
				
				//Test Climber
				addSequential(new ClimberPercentCommand(Climber.CLIMB_PERCENT));
				addSequential(new WaitCommand(0.25));
				addSequential(new ClimberPercentCommand(0));
			}
		});
	}
}
