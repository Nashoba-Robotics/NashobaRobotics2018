package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.multicommands.DriveToCubeCommandAdvanced;
import edu.nr.robotics.multicommands.PrepareCubeIntakeCommand;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class ScaleToBlockProfilingCommand extends CommandGroup {

	public ScaleToBlockProfilingCommand(int block) {

		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				
				addParallel(new PrepareCubeIntakeCommand());
				
				addParallel(new AnonymousCommandGroup() {
					
					@Override
					public void commands() {
						/*addSequential(new EnableMotionProfile(FieldMeasurements.PIVOT_POINT_TO_SCALE_DIAGONAL.negate(), Distance.ZERO,
								Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));*/

						addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
								(FieldMeasurements.PIVOT_POINT_TO_SCALE.add(FieldMeasurements.PIVOT_POINT_TO_CUBE_1)),
								Drive.MAX_PROFILE_TURN_PERCENT)) {

							@Override
							protected boolean condition() {
								return block == 1;
							}

						});
						
						addSequential(new EnableLimelightCommand(true));
						
						addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
								FieldMeasurements.PIVOT_POINT_TO_SCALE.add(FieldMeasurements.PIVOT_POINT_TO_CUBE_1).negate(),
								Drive.MAX_PROFILE_TURN_PERCENT)) {

							@Override
							protected boolean condition() {
								return block == 6;
							}

						});
						
						addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
								FieldMeasurements.PIVOT_POINT_TO_SCALE.add(FieldMeasurements.PIVOT_POINT_TO_CUBE_2),
								Drive.MAX_PROFILE_TURN_PERCENT)) {

							@Override
							protected boolean condition() {
								return block == 2;
							}

						});

						addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
								FieldMeasurements.PIVOT_POINT_TO_SCALE.add(FieldMeasurements.PIVOT_POINT_TO_CUBE_3),
								Drive.MAX_PROFILE_TURN_PERCENT)) {

							@Override
							protected boolean condition() {
								return block == 3;
							}

						});
						
						addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
								(FieldMeasurements.PIVOT_POINT_TO_SCALE.add(FieldMeasurements.PIVOT_POINT_TO_CUBE_2))
										.negate(),
								Drive.MAX_PROFILE_TURN_PERCENT)) {

							@Override
							protected boolean condition() {
								return block == 5;
							}

						});

						addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
								(FieldMeasurements.PIVOT_POINT_TO_SCALE.add(FieldMeasurements.PIVOT_POINT_TO_CUBE_3))
										.negate(),
								Drive.MAX_PROFILE_TURN_PERCENT)) {

							@Override
							protected boolean condition() {
								return block == 4;
							}

						});

					}
				});
				
			}
		});
		
		addSequential(new DriveToCubeCommandAdvanced());
		
		addSequential(new EnableLimelightCommand(false));
		
	}

}
