package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class BlockToScaleProfilingCommand extends CommandGroup {

	public BlockToScaleProfilingCommand(int block) {

		addSequential(new ConditionalCommand(new EnableMotionProfile(FieldMeasurements.CUBE_1_TO_PIVOT_POINT_DIAGONAL.negate(), Distance.ZERO,
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {

					@Override
			protected boolean condition() {
				return (block == 1 && FieldData.getInstance().scale == Direction.left)
						|| (block == 6 && FieldData.getInstance().scale == Direction.right);
					}
			
		});
		
		addSequential(new ConditionalCommand(new EnableMotionProfile(FieldMeasurements.CUBE_TO_PLATFORM_ZONE_DIAGONAL, Distance.ZERO,
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {

					@Override
			protected boolean condition() {
				return (block == 1 && FieldData.getInstance().scale == Direction.right)
						|| (block == 6 && FieldData.getInstance().scale == Direction.left);
					}
			
		});
		
		addSequential(new ConditionalCommand(new EnableMotionProfile(FieldMeasurements.CUBE_2_TO_PIVOT_POINT_DIAGONAL.negate(), Distance.ZERO,
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {

					@Override
					protected boolean condition() {
						return block == 2 || block == 5;
					}
			
		});
		
		addSequential(new ConditionalCommand(new EnableMotionProfile(FieldMeasurements.CUBE_3_TO_PIVOT_POINT_DIAGONAL.negate(), Distance.ZERO,
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {

					@Override
					protected boolean condition() {
						return block == 3 || block == 4;
					}
			
		});
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
				(FieldMeasurements.PIVOT_POINT_TO_SCALE.add(FieldMeasurements.PIVOT_POINT_TO_CUBE_1)).negate(),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 1 && FieldData.getInstance().scale == Direction.left;
			}

		});
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
				FieldMeasurements.PIVOT_POINT_TO_CUBE_1.negate(), Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 1 && FieldData.getInstance().scale == Direction.right;
			}

		});
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
				(FieldMeasurements.PIVOT_POINT_TO_SCALE.add(FieldMeasurements.PIVOT_POINT_TO_CUBE_2)).negate(),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 2;
			}

		});
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
				(FieldMeasurements.PIVOT_POINT_TO_SCALE.add(FieldMeasurements.PIVOT_POINT_TO_CUBE_3)).negate(),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 3;
			}

		});	
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
				(FieldMeasurements.PIVOT_POINT_TO_SCALE.add(FieldMeasurements.PIVOT_POINT_TO_CUBE_3)),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 4;
			}

		});	
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
				(FieldMeasurements.PIVOT_POINT_TO_SCALE.add(FieldMeasurements.PIVOT_POINT_TO_CUBE_2)),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 5;
			}

		});	
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
				FieldMeasurements.PIVOT_POINT_TO_SCALE.add(FieldMeasurements.PIVOT_POINT_TO_CUBE_1),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 6 && FieldData.getInstance().scale == Direction.right;
			}

		});
		
		addSequential(new ConditionalCommand(new EnableMotionProfile(FieldMeasurements.PIVOT_POINT_TO_SCALE_DIAGONAL, Distance.ZERO,
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {

					@Override
					protected boolean condition() {
						return block == 1 || block == 6;
					}
		});
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
				FieldMeasurements.PIVOT_POINT_TO_CUBE_1, Drive.MAX_PROFILE_TURN_PERCENT, true)) {

					@Override
					protected boolean condition() {
						return block == 6 && FieldData.getInstance().scale == Direction.left;
					}
		});
		
		addSequential(new ConditionalCommand(new EnableMotionProfile(FieldMeasurements.PLATFORM_ZONE_WIDTH_BLOCK,
				Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {

			@Override
			protected boolean condition() {
				return (block == 6 && FieldData.getInstance().scale == Direction.left)
						|| (block == 1 && FieldData.getInstance().scale == Direction.right);
			}

		});
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
				(new Angle(180, Angle.Unit.DEGREE).sub(FieldMeasurements.PIVOT_POINT_TO_SCALE)).negate(), Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return (block == 1 && FieldData.getInstance().scale == Direction.right);
			}

		});

		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(),
				new Angle(180, Angle.Unit.DEGREE).sub(FieldMeasurements.PIVOT_POINT_TO_SCALE), Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return (block == 6 && FieldData.getInstance().scale == Direction.left);
			}

		});
		
		addSequential(new ConditionalCommand(
				new EnableMotionProfile(FieldMeasurements.PLATFORM_ZONE_TO_SCALE_DIAGONAL, Distance.ZERO,
						Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT),
				new EnableMotionProfile(FieldMeasurements.PIVOT_POINT_TO_SCALE_DIAGONAL, Distance.ZERO,
						Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {

			@Override
			protected boolean condition() {
				return (block == 6 && FieldData.getInstance().scale == Direction.left)
						|| (block == 1 && FieldData.getInstance().scale == Direction.right);
			}
			
		});

		addSequential(new EnableMotionProfile(FieldMeasurements.PIVOT_POINT_TO_SCALE_DIAGONAL, Distance.ZERO,
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));

	}

}
