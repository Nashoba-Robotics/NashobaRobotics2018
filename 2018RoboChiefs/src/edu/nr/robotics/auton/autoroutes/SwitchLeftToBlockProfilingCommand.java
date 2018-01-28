package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnPIDCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.nr.robotics.auton.FieldDistances;
import edu.nr.robotics.multicommands.DriveToCubeCommand;;

public class SwitchLeftToBlockProfilingCommand extends CommandGroup {

	public SwitchLeftToBlockProfilingCommand() {
		
		//If strafing
			//addSequential(new EnableMotionProfile(FieldDistances.SWITCH_TO_PLATFORM_ZONE_Y, FieldDistances.SWITCH_TO_PLATFORM_ZONE_X.negate(), Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		//If not strafing
			addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(-90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
		
			addSequential(new EnableMotionProfile(FieldDistances.SWITCH_TO_PLATFORM_ZONE_X, FieldDistances.SWITCH_TO_PLATFORM_ZONE_Y, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
			addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
		
		addSequential(new EnableMotionProfile(FieldDistances.FAR_START_POS_TO_OUTER_BLOCK_Y, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
			
		addSequential(new DriveToCubeCommand());
		
		addSequential(new EnableMotionProfile(FieldDistances.CUBE_TO_PLATFORM_ZONE_X.negate(), Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
	}
	
}
