package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnPIDCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class StartPosFarRightToScaleRightProfilingCommand extends CommandGroup {
	
	public StartPosFarRightToScaleRightProfilingCommand () {
	
		addSequential(new EnableMotionProfile(FieldMeasurements.BASELINE_TO_SCALE_X, Distance.ZERO, 
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnPIDCommand(Drive.getInstance(), (new Angle(90, Angle.Unit.DEGREE).sub(FieldMeasurements.PLATFORM_ZONE_TO_SCALE)).negate(), 
				Drive.MAX_PROFILE_TURN_PERCENT, true));
	}

}
