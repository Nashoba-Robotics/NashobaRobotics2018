package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Switch;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnPIDCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosLeftToScaleRightProfilingCommand extends CommandGroup {

	public StartPosLeftToScaleRightProfilingCommand() {
		
		
		addSequential(new EnableMotionProfile(FieldMeasurements.BASELINE_TO_PLATFORM_ZONE_X, FieldMeasurements.BASELINE_TO_PLATFORM_ZONE_Y, 
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
		
		addSequential(new EnableMotionProfile(FieldMeasurements.PLATFORM_ZONE_WIDTH_SHORT, Distance.ZERO, 
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
			
		addSequential(new TurnPIDCommand(Drive.getInstance(), (new Angle(180, Angle.Unit.DEGREE).sub(FieldMeasurements.PLATFORM_ZONE_TO_SCALE)).negate(), 
				Drive.MAX_PROFILE_TURN_PERCENT, true));
		
		addSequential(new EnableMotionProfile(FieldMeasurements.PLATFORM_ZONE_TO_SCALE_DIAGONAL, Distance.ZERO, 
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
	}
	
}
