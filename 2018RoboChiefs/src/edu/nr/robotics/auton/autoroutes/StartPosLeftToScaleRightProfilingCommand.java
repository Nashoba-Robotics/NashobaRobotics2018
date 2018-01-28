package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldDistances;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnPIDCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosLeftToScaleRightProfilingCommand extends CommandGroup {

	public StartPosLeftToScaleRightProfilingCommand() {
		
		addSequential(new EnableMotionProfile(FieldDistances.BASELINE_TO_PLATFORM_ZONE_X, FieldDistances.BASELINE_TO_PLATFORM_ZONE_Y, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(90,Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
		
		addSequential(new EnableMotionProfile(FieldDistances.PLATFORM_ZONE_WIDTH, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		//If not strafing
		addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(-90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
		
		addSequential(new EnableMotionProfile(FieldDistances.PLATFORM_ZONE_TO_SCALE_X, FieldDistances.PLATFORM_ZONE_TO_SCALE_Y, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(-90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
		
		//If Strafing
		/*addSequential(new EnableMotionProfile(FieldDistances.PLATFORM_ZONE_TO_SCALE_Y, FieldDistances.PLATFORM_ZONE_TO_SCALE_X.negate(), Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(180, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
		 */
	}
	
}
