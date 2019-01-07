package edu.nr.robotics.auton;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class DriveOverBaselineFancyCommand extends CommandGroup {
	
	public DriveOverBaselineFancyCommand() {
		
		addSequential(new EnableMotionProfile(FieldMeasurements.BASELINE_TO_PLATFORM_ZONE_X, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new ConditionalCommand(new TurnCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT), new TurnCommand(Drive.getInstance(), new Angle(-90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.right;
			}
			
		});
		
		addSequential(new EnableMotionProfile(FieldMeasurements.PLATFORM_ZONE_WIDTH_START_POS.mul(0.5), Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
	}

}
