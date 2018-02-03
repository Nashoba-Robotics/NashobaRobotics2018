package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosRightToSwitchRightProfilingCommand extends CommandGroup {

	public StartPosRightToSwitchRightProfilingCommand() {
		
		addSequential(new DriveOverBaselineAutoCommand());
		
	}
	
}
