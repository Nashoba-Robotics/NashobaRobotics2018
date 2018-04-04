package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorBottomCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersIntakeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class SwitchToCube2ProfilingCommand extends CommandGroup {
	
	public SwitchToCube2ProfilingCommand() {
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				//addParallel(new ElevatorBottomCommand());
				
				addParallel(new EnableMotionProfile(
						(FieldMeasurements.CUBE_1_TO_SWITCH_DIAGONAL.add(FieldMeasurements.CUBE_1_TO_PIVOT_POINT_DIAGONAL)).negate(),
						Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
			
				/*addParallel(new AnonymousCommandGroup() {
					
					@Override
					public void commands() {
						addSequential(new WaitCommand(0.5));
						addSequential(new IntakeElevatorBottomCommand());
					}
				});*/
			}
		});
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(), FieldMeasurements.CUBE_1_TO_CUBE_2, Drive.MAX_PROFILE_TURN_PERCENT), new TurnCommand(Drive.getInstance(), FieldMeasurements.CUBE_1_TO_CUBE_2.negate(), Drive.MAX_PROFILE_TURN_PERCENT)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.right;
			}
			
		});
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				//addParallel(new IntakeRollersIntakeCommand());
				addParallel(new EnableMotionProfile(FieldMeasurements.CUBE_2_TO_PIVOT_POINT_DIAGONAL, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));

			}
		});
				
	}

}
