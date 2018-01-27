package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosRightToScaleLeftProfilingCommand extends CommandGroup {

	public StartPosRightToScaleLeftProfilingCommand() {
		
		addSequential(new EnableMotionProfile(Distance.ZERO, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new EnableMotionProfile(Distance.ZERO, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new EnableMotionProfile(Distance.ZERO, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
	}
	
}
