package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosFarRightToScaleLeftProfilingCommand extends CommandGroup {

	public StartPosFarRightToScaleLeftProfilingCommand() {
	
		addSequential(new EnableMotionProfile(FieldMeasurements.BASELINE_TO_PLATFORM_ZONE_X, FieldMeasurements.BASELINE_TO_PLATFORM_ZONE_Y, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnCommand(Drive.getInstance(), new Angle(-90,Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
		
		addSequential(new EnableMotionProfile(FieldMeasurements.PLATFORM_ZONE_WIDTH_START_POS, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnCommand(Drive.getInstance(), new Angle(180, Angle.Unit.DEGREE).sub(FieldMeasurements.PIVOT_POINT_TO_SCALE), Drive.MAX_PROFILE_TURN_PERCENT, true));
		
		addSequential(new EnableMotionProfile(FieldMeasurements.PIVOT_POINT_TO_SCALE_DIAGONAL, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
	}
	
}
