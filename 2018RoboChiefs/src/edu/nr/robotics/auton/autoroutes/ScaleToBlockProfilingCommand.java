package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.StartPos;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.drive.TurnToCubeCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorBottomCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersIntakeCommand;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.PrintCommand;

public class ScaleToBlockProfilingCommand extends CommandGroup {

	public ScaleToBlockProfilingCommand(int block) {
		
		addSequential(new EnableLimelightCommand(true));
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				//addParallel(new ElevatorBottomCommand());
				
				addParallel(new AnonymousCommandGroup() {
					
					@Override
					public void commands() {
						
						addParallel(new IntakeElevatorBottomCommand());
						
						addParallel(new IntakeRollersIntakeCommand());
						
						addParallel(new AnonymousCommandGroup() {
							
							@Override
							public void commands() {
								
								addSequential(new TurnToCubeCommand());
								
								addSequential(new ConditionalCommand(new EnableMotionProfile(FieldMeasurements.CUBE_1_TO_PIVOT_POINT_DIAGONAL, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {
									
									@Override
									protected boolean condition() {
										return block == 1 || block == 6;
									}
								});
								
								addSequential(new ConditionalCommand(new EnableMotionProfile(FieldMeasurements.CUBE_2_TO_PIVOT_POINT_DIAGONAL, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {
									
									@Override
									protected boolean condition() {
										return block == 2 || block == 5;
									}
								});
								
								addSequential(new ConditionalCommand(new EnableMotionProfile(FieldMeasurements.CUBE_3_TO_PIVOT_POINT_DIAGONAL, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {
									
									@Override
									protected boolean condition() {
										return block == 3 || block == 4;
									}
								});
								
							}
						});
						
					}
				});
				
			}
			
		});
		
		addSequential(new EnableLimelightCommand(false));
		
	}

}
