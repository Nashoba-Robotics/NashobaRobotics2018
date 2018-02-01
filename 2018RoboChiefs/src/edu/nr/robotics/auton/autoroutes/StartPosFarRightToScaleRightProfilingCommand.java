package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosFarRightToScaleRightProfilingCommand extends CommandGroup {
	
	public StartPosFarRightToScaleRightProfilingCommand () {

		addSequential(new EnableMotionProfile(FieldMeasurements.BASELINE_TO_SCALE_X, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnCommand(Drive.getInstance(), (new Angle(90, Angle.Unit.DEGREE).sub(FieldMeasurements.PIVOT_POINT_TO_SCALE)).negate(), Drive.MAX_PROFILE_TURN_PERCENT, true));
	}

}
