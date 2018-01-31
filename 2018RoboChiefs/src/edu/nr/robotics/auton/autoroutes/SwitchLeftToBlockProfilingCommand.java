package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.multicommands.DriveToCubeCommandAdvanced;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnPIDCommand;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;;

public class SwitchLeftToBlockProfilingCommand extends CommandGroup {

	public SwitchLeftToBlockProfilingCommand() {

		addSequential(new EnableMotionProfile(
				(FieldMeasurements.SWITCH_EDGE_TO_FIELD_EDGE_Y.sub(FieldMeasurements.FAR_START_POS_TO_FIELD_EDGE_Y)
						.sub(FieldMeasurements.ROBOT_LENGTH.mul(0.5))).negate(),
				Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));

		addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE),
				Drive.MAX_PROFILE_TURN_PERCENT, true));

		addSequential(new EnableMotionProfile(FieldMeasurements.SWITCH_TO_PLATFORM_ZONE_X.negate(),
				Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));

		addSequential(new EnableLimelightCommand(true));
		
		addSequential(new TurnPIDCommand(Drive.getInstance(),
				(new Angle(90, Angle.Unit.DEGREE).sub(FieldMeasurements.PLATFORM_ZONE_TO_CUBE)).negate(),
				Drive.MAX_PROFILE_TURN_PERCENT, true));

		addSequential(new DriveToCubeCommandAdvanced());

		addSequential(new EnableLimelightCommand(false));
	}

}
