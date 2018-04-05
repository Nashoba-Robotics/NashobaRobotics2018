package edu.nr.robotics.auton.automap;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.multicommands.PrepareScoreSwitchAutoCommand;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterShootCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeDeployCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class AutoSwitchLoopFrontCommand extends CommandGroup {

	public AutoSwitchLoopFrontCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				//addParallel(new ElevatorBottomCommand());
				
				addParallel(new EnableMotionProfile(
						(FieldMeasurements.BASELINE_TO_SWITCH_X.sub(FieldMeasurements.BASELINE_TO_10_CUBE_PILE)
								.sub(FieldMeasurements.ROBOT_LENGTH.mul(0.5)).negate()),
						Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
				
			}
		});
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				//addParallel(new IntakeDeployCommand());
				
				addParallel(new AnonymousCommandGroup() {
					
					@Override
					public void commands() {
						
						addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(), FieldMeasurements.FRONT_SWITCH_LOOP_TO_CUBE_MID_ANGLE, Drive.MAX_PROFILE_TURN_PERCENT), new TurnCommand(Drive.getInstance(), FieldMeasurements.FRONT_SWITCH_LOOP_TO_CUBE_MID_ANGLE.negate(), Drive.MAX_PROFILE_TURN_PERCENT)) {
							
							@Override
							protected boolean condition() {
								return FieldData.getInstance().nearSwitch == Direction.left;
							}
							
						});
						
					}
				});
				
			}
		});
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				//addParallel(new IntakeRollersIntakeCommand());
				addParallel(new EnableMotionProfile(FieldMeasurements.FRONT_SWITCH_LOOP_TO_CUBE_MID_DIAGONAL, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
				
			}
		});
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				//addParallel(new PrepareScoreSwitchAutoCommand());
				
				addParallel(new AnonymousCommandGroup() {
					
					@Override
					public void commands() {
						
						addSequential(new EnableMotionProfile(FieldMeasurements.FRONT_SWITCH_LOOP_TO_CUBE_MID_DIAGONAL.negate(), Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
						
						addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(), FieldMeasurements.FRONT_SWITCH_LOOP_TO_CUBE_MID_ANGLE.negate(), Drive.MAX_PROFILE_TURN_PERCENT), new TurnCommand(Drive.getInstance(), FieldMeasurements.FRONT_SWITCH_LOOP_TO_CUBE_MID_ANGLE, Drive.MAX_PROFILE_TURN_PERCENT)) {
							
							@Override
							protected boolean condition() {
								return FieldData.getInstance().nearSwitch == Direction.left;
							}
							
						});
					}
				});
			}
		});
		
		addSequential(new EnableMotionProfile(
				(FieldMeasurements.BASELINE_TO_SWITCH_X.sub(FieldMeasurements.BASELINE_TO_10_CUBE_PILE)
						.sub(FieldMeasurements.ROBOT_LENGTH.mul(0.5))),
				Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		//addSequential(new ElevatorProfileCommandGroup(Elevator.SWITCH_HEIGHT_ELEVATOR, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
		
		//addSequential(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_HIGH_ELEVATOR_SHOOTER));
		
		//Two cube stops here
		/////////////////////
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				//addParallel(new ElevatorBottomCommand());
				
				addParallel(new EnableMotionProfile(
						(FieldMeasurements.BASELINE_TO_SWITCH_X.sub(FieldMeasurements.BASELINE_TO_10_CUBE_PILE)
								.sub(FieldMeasurements.ROBOT_LENGTH.mul(0.5)).negate()),
						Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
				
			}
		});
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				//addParallel(new IntakeDeployCommand());
				
				addParallel(new AnonymousCommandGroup() {
					
					@Override
					public void commands() {
						
						addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(), FieldMeasurements.FRONT_SWITCH_LOOP_TO_CUBE_FAR_ANGLE, Drive.MAX_PROFILE_TURN_PERCENT), new TurnCommand(Drive.getInstance(), FieldMeasurements.FRONT_SWITCH_LOOP_TO_CUBE_FAR_ANGLE.negate(), Drive.MAX_PROFILE_TURN_PERCENT)) {
							
							@Override
							protected boolean condition() {
								return FieldData.getInstance().nearSwitch == Direction.left;
							}
							
						});
						
					}
				});
				
			}
		});
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				//addParallel(new IntakeRollersIntakeCommand());
				addParallel(new EnableMotionProfile(FieldMeasurements.FRONT_SWITCH_LOOP_TO_CUBE_FAR_DIAGONAL, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
				
			}
		});
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				//addParallel(new PrepareScoreSwitchAutoCommand());
				
				addParallel(new AnonymousCommandGroup() {
					
					@Override
					public void commands() {
						
						addSequential(new EnableMotionProfile(FieldMeasurements.FRONT_SWITCH_LOOP_TO_CUBE_FAR_DIAGONAL.negate(), Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
						
						addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(), FieldMeasurements.FRONT_SWITCH_LOOP_TO_CUBE_FAR_ANGLE.negate(), Drive.MAX_PROFILE_TURN_PERCENT), new TurnCommand(Drive.getInstance(), FieldMeasurements.FRONT_SWITCH_LOOP_TO_CUBE_FAR_ANGLE, Drive.MAX_PROFILE_TURN_PERCENT)) {
							
							@Override
							protected boolean condition() {
								return FieldData.getInstance().nearSwitch == Direction.left;
							}
							
						});
					}
				});
			}
		});
		
		addSequential(new EnableMotionProfile(
				(FieldMeasurements.BASELINE_TO_SWITCH_X.sub(FieldMeasurements.BASELINE_TO_10_CUBE_PILE)
						.sub(FieldMeasurements.ROBOT_LENGTH.mul(0.5))),
				Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		//addSequential(new ElevatorProfileCommandGroup(Elevator.SWITCH_HEIGHT_ELEVATOR, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
		
		//addSequential(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_HIGH_ELEVATOR_SHOOTER));
		
	}
}
