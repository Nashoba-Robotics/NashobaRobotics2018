package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterShootCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeDeployCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class StartPosLeftToScaleLeftProfilingCommand extends CommandGroup {

public StartPosLeftToScaleLeftProfilingCommand() {
		
		addParallel(new IntakeDeployCommand());
						
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				addParallel(new AnonymousCommandGroup() {
					
					@Override
					public void commands() {
						
						addSequential(new WaitCommand(FieldMeasurements.AUTO_ELEVATOR_LIFT_WAIT_TIME.get(Time.Unit.SECOND)));
						addSequential(new ElevatorProfileCommandGroup(Elevator.SCALE_HEIGHT_ELEVATOR, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
						
					}
				});
				
				addParallel(new EnableMotionProfile(FieldMeasurements.BASELINE_TO_SCALE_X, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
												
			}
		});
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				addParallel(new AnonymousCommandGroup() {
					
					@Override
					public void commands() {
					
						addSequential(new WaitCommand(FieldMeasurements.AUTO_SHOOT_TURNING_FIRST_WAIT_TIME.get(Time.Unit.SECOND)));
						addSequential(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_SCALE_AUTO_ELEVATOR_SHOOTER));
						addSequential(new ElevatorBottomCommand());
					}
				});
				
				addSequential(new TurnCommand(Drive.getInstance(),
						new Angle(90, Angle.Unit.DEGREE).add(FieldMeasurements.PIVOT_POINT_TO_CUBE_1),
						Drive.MAX_PROFILE_TURN_PERCENT));	
			}
			
		});

		
	}
	
}
