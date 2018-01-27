package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosMiddleToSwitchLeftProfilingCommand extends CommandGroup {

	public StartPosMiddleToSwitchLeftProfilingCommand() {
		
		addSequential(new EnableMotionProfile(Distance.ZERO, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
	}
	
}
