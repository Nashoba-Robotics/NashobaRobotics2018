package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.NRMath;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnPIDCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.multicommands.DriveToCubeCommandAdvanced;;

public class SwitchLeftToBlockProfilingCommand extends CommandGroup {

	public SwitchLeftToBlockProfilingCommand() {

		addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE),
				Drive.MAX_PROFILE_TURN_PERCENT, true));

		addSequential(new EnableMotionProfile(FieldMeasurements.SWITCH_TO_PLATFORM_ZONE_X.negate(),
				FieldMeasurements.SWITCH_TO_PLATFORM_ZONE_Y, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));

		addSequential(new TurnPIDCommand(Drive.getInstance(),
				(new Angle(90, Angle.Unit.DEGREE).sub(FieldMeasurements.PLATFORM_ZONE_TO_CUBE)).negate(),
				Drive.MAX_PROFILE_TURN_PERCENT, true));

		addSequential(new DriveToCubeCommandAdvanced());

	}

}
