package edu.nr.robotics.auton.automap;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.auton.autoroutes.PlatformZoneSwitchLeftToBlockProfilingCommand;
import edu.nr.robotics.auton.autoroutes.PlatformZoneSwitchRightToBlockProfilingCommand;
import edu.nr.robotics.multicommands.PrepareCubeIntakeCommand;
import edu.nr.robotics.multicommands.PrepareScoreElevatorBottomCommand;
import edu.nr.robotics.multicommands.PrepareScoreSwitchAutoCommand;
import edu.nr.robotics.multicommands.PrepareScoreSwitchCommand;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveCurrentCommand;
import edu.nr.robotics.subsystems.drive.DriveToCubeCommandAdvanced;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.StrafeToCubeCommand;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.drive.TurnToCubeCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomDropCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterShootCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorFoldCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class AutoSwitchLoopCommand extends CommandGroup {

	public AutoSwitchLoopCommand() {
						
		addSequential(new ConditionalCommand(new PlatformZoneSwitchLeftToBlockProfilingCommand(), new PlatformZoneSwitchRightToBlockProfilingCommand()) {
			
			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});
				
		addSequential(new PrepareScoreSwitchAutoCommand());
				
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(), FieldMeasurements.SWITCH_LOOP_SWITCH_ANGLE.negate(), Drive.MAX_PROFILE_TURN_PERCENT), new TurnCommand(Drive.getInstance(), FieldMeasurements.SWITCH_LOOP_SWITCH_ANGLE, Drive.MAX_PROFILE_TURN_PERCENT)) {

			@Override
			protected boolean condition() {				
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});
		
		addSequential(new ElevatorShooterShootCommand(ElevatorShooter.shootPercent));
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(), FieldMeasurements.SWITCH_LOOP_SWITCH_ANGLE, Drive.MAX_PROFILE_TURN_PERCENT), new TurnCommand(Drive.getInstance(), FieldMeasurements.SWITCH_LOOP_SWITCH_ANGLE.negate(), Drive.MAX_PROFILE_TURN_PERCENT)) {

			@Override
			protected boolean condition() {				
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				addParallel(new PrepareCubeIntakeCommand());
				
				addParallel(new EnableMotionProfile(FieldMeasurements.CUBE_TO_PLATFORM_ZONE_X.negate(), Distance.ZERO,
						Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
				
			}
		});
		
		addSequential(new ConditionalCommand(new StrafeToCubeCommand(Direction.left), new StrafeToCubeCommand(Direction.right)) {
			
			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});
		
		addSequential(new AnonymousCommandGroup() {

			@Override
			public void commands() {
				
				addParallel(new IntakeRollersIntakeCommand());
				
				/*addParallel(new DriveToCubeCommandAdvanced());*/
				
				addParallel(new AnonymousCommandGroup() {
					
					@Override
					public void commands() {
						addSequential(new TurnToCubeCommand());
						addSequential(new EnableMotionProfile(FieldMeasurements.CUBE_TO_PLATFORM_ZONE_X, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
						
					}
				});	
				
			}
			
		});
		
		addSequential(new PrepareScoreSwitchAutoCommand());
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(), FieldMeasurements.SWITCH_LOOP_SWITCH_ANGLE, Drive.MAX_PROFILE_TURN_PERCENT), new TurnCommand(Drive.getInstance(), FieldMeasurements.SWITCH_LOOP_SWITCH_ANGLE.negate(), Drive.MAX_PROFILE_TURN_PERCENT)) {

			@Override
			protected boolean condition() {				
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});
		
		addSequential(new ElevatorShooterShootCommand(ElevatorShooter.shootPercent));
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(), FieldMeasurements.SWITCH_LOOP_SWITCH_ANGLE.negate(), Drive.MAX_PROFILE_TURN_PERCENT), new TurnCommand(Drive.getInstance(), FieldMeasurements.SWITCH_LOOP_SWITCH_ANGLE, Drive.MAX_PROFILE_TURN_PERCENT)) {

			@Override
			protected boolean condition() {				
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				addParallel(new PrepareCubeIntakeCommand());
				
				addParallel(new EnableMotionProfile(FieldMeasurements.CUBE_TO_PLATFORM_ZONE_X.negate(), Distance.ZERO,
						Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
				
			}
		});
		
		addSequential(new ConditionalCommand(new StrafeToCubeCommand(Direction.left), new StrafeToCubeCommand(Direction.right)) {
			
			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});
	
		addSequential(new AnonymousCommandGroup() {

			@Override
			public void commands() {
				
				addParallel(new PrepareScoreElevatorBottomCommand());
				
				/*addParallel(new DriveToCubeCommandAdvanced());*/
				
				addParallel(new AnonymousCommandGroup() {
					
					@Override
					public void commands() {
						addSequential(new TurnToCubeCommand());
						addSequential(new EnableMotionProfile(FieldMeasurements.CUBE_TO_PLATFORM_ZONE_X, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
						
					}
				});	
				
			}
			
		});
		
		addSequential(new IntakeElevatorFoldCommand());

		
	}
	
}
