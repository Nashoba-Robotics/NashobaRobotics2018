package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableTwoDMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosLeftToScaleLeftProfilingCommand extends CommandGroup {

	public StartPosLeftToScaleLeftProfilingCommand() {

		addSequential(new EnableTwoDMotionProfile(FieldMeasurements.BASELINE_TO_SCALE_X, Distance.ZERO,
				FieldMeasurements.PIVOT_POINT_TO_SCALE.negate(), Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT,
				"StartPosLeftToScaleLeftProfile"));

	}

}
