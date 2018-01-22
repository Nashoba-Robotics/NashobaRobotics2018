package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableOneDMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosLeftToSwitchRightProfilingCommand extends CommandGroup {

	public StartPosLeftToSwitchRightProfilingCommand() {
		
		addSequential(new EnableOneDMotionProfile(Distance.ZERO, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
	}
	
}
