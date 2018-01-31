package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnPIDCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class BlockToScaleProfilingCommand extends CommandGroup {

	public BlockToScaleProfilingCommand(int block) {

		addSequential(new ConditionalCommand(new EnableMotionProfile(FieldMeasurements.CUBE_TO_PLATFORM_ZONE_DIAGONAL.negate(), Distance.ZERO,
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {

					@Override
					protected boolean condition() {
						return block == 1 || block == 6;
					}
			
		});
		
		addSequential(new ConditionalCommand(new TurnPIDCommand(Drive.getInstance(),
				(FieldMeasurements.PLATFORM_ZONE_TO_SCALE.add(FieldMeasurements.PLATFORM_ZONE_TO_CUBE)).negate(),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 1 && FieldData.getInstance().scale == Direction.left;
			}

		});
		
		addSequential(new ConditionalCommand(new TurnPIDCommand(Drive.getInstance(),
				FieldMeasurements.PLATFORM_ZONE_TO_SCALE.add(FieldMeasurements.PLATFORM_ZONE_TO_CUBE),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 6 && FieldData.getInstance().scale == Direction.right;
			}

		});
		
		addSequential(new ConditionalCommand(new EnableMotionProfile(FieldMeasurements.PLATFORM_ZONE_TO_SCALE_DIAGONAL, Distance.ZERO,
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {

					@Override
					protected boolean condition() {
						return block == 1 || block == 6;
					}
		});

	}

}
