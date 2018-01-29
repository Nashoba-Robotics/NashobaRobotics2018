package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.FieldDistances;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnPIDCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class BlockToScaleProfilingCommand extends CommandGroup {

	public BlockToScaleProfilingCommand(int block) {

		addSequential(new ConditionalCommand(new TurnPIDCommand(Drive.getInstance(), new Angle(180, Angle.Unit.DEGREE),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 1 || block == 6;
			}

		});

		addSequential(new ConditionalCommand(new TurnPIDCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return (block == 2 || block == 3 || block == 4 || block == 5)
						&& FieldData.getInstance().scale == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new TurnPIDCommand(Drive.getInstance(), new Angle(-90, Angle.Unit.DEGREE),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return (block == 2 || block == 3 || block == 4 || block == 5)
						&& FieldData.getInstance().scale == Direction.left;
			}

		});
		
		addSequential(new EnableMotionProfile(FieldDistances.CUBE_TO_SCALE_LOOP_X, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));

		/*
		 * addSequential(new ConditionalCommand(new
		 * TurnPIDCommand(Drive.getInstance(), new Angle((Math.PI / 2) +
		 * Math.atan(FieldDistances.PLATFORM_ZONE_TO_SCALE_X.div(FieldDistances.
		 * FAR_START_POS_TO_OUTER_BLOCK_Y)), Angle.Unit.RADIAN),
		 * Drive.MAX_PROFILE_TURN_PERCENT, true)){
		 * 
		 * @Override protected boolean condition() { return
		 * FieldData.getInstance().scale == Direction.left; }
		 * 
		 * });
		 * 
		 * addSequential(new ConditionalCommand(new
		 * TurnPIDCommand(Drive.getInstance(), new Angle(-(Math.PI / 2) -
		 * Math.atan(FieldDistances.PLATFORM_ZONE_TO_SCALE_X.div(FieldDistances.
		 * FAR_START_POS_TO_OUTER_BLOCK_Y)), Angle.Unit.RADIAN),
		 * Drive.MAX_PROFILE_TURN_PERCENT, true)){
		 * 
		 * @Override protected boolean condition() { return
		 * FieldData.getInstance().scale == Direction.right; }
		 * 
		 * });
		 * 
		 * addSequential(new ConditionalCommand(new
		 * TurnPIDCommand(Drive.getInstance(), new Angle((Math.PI) -
		 * Math.atan(FieldDistances.PLATFORM_ZONE_TO_SCALE_X.div(FieldDistances.
		 * FAR_START_POS_TO_OUTER_BLOCK_Y)), Angle.Unit.RADIAN),
		 * Drive.MAX_PROFILE_TURN_PERCENT, true)){
		 * 
		 * @Override protected boolean condition() { return block == 1 || block
		 * == 2 || block == 3; }
		 * 
		 * });
		 * 
		 * addSequential(new ConditionalCommand(new
		 * TurnPIDCommand(Drive.getInstance(), new Angle(-(Math.PI) +
		 * Math.atan(FieldDistances.PLATFORM_ZONE_TO_SCALE_X.div(FieldDistances.
		 * FAR_START_POS_TO_OUTER_BLOCK_Y)), Angle.Unit.RADIAN),
		 * Drive.MAX_PROFILE_TURN_PERCENT, true)){
		 * 
		 * @Override protected boolean condition() { return block == 6 || block
		 * == 5 || block == 4; }
		 * 
		 * });
		 * 
		 * addSequential(new EnableMotionProfile(NRMath.hypot(FieldDistances.
		 * FAR_START_POS_TO_OUTER_BLOCK_Y,
		 * FieldDistances.PLATFORM_ZONE_TO_SCALE_X), Distance.ZERO,
		 * Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		 * 
		 * addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(0,
		 * Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
		 */
	}

}
