package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.multicommands.DriveToCubeCommandAdvanced;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.StrafeToCubeCommand;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PlatformZoneSwitchRightToBlockProfilingCommand extends CommandGroup {

	public PlatformZoneSwitchRightToBlockProfilingCommand() {
		
		addSequential(new EnableMotionProfile(
				(FieldMeasurements.SWITCH_EDGE_TO_FIELD_EDGE_Y.sub(FieldMeasurements.PIVOT_POINT_FIELD_EDGE_Y)
						.sub(FieldMeasurements.ROBOT_LENGTH.mul(0.5))).negate(),
				Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));

		addSequential(new TurnCommand(Drive.getInstance(), new Angle(-90, Angle.Unit.DEGREE),
				Drive.MAX_PROFILE_TURN_PERCENT, true));
		
		addSequential(new EnableMotionProfile(
				(FieldMeasurements.BASELINE_TO_PLATFORM_ZONE_X.sub(FieldMeasurements.BASELINE_TO_SWITCH_X)).negate(),
				Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new StrafeToCubeCommand(Direction.right));
		
		addSequential(new DriveToCubeCommandAdvanced());

	}
	
}
