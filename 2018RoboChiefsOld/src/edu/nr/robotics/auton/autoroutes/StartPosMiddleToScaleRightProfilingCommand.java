package edu.nr.robotics.auton.autoroutes;

import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosMiddleToScaleRightProfilingCommand extends CommandGroup {
	
	public StartPosMiddleToScaleRightProfilingCommand() {
		
		addSequential(new DriveOverBaselineAutoCommand());
		
	}

}
