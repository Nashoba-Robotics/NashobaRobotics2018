package edu.nr.robotics.auton;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.Robot;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveForwardBasicCommand;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class DriveOverBaselineAutoCommand extends CommandGroup {

	/**
	 * The distance (in inches) to drive to get over baseline safely in auto.
	 * 
	 * It needs to be 120 but can be 288 - robotLength. 196 Gets back of robot to back of switch
	 */
	public static final Distance DISTANCE_TO_GET_OVER_BASELINE = new Distance(140, Distance.Unit.INCH); //TODO: Put in distance to baseline
	//196 for next to open area
	//140 for next to switch
	
	
	public DriveOverBaselineAutoCommand() {

		
		addSequential(new ConditionalCommand(new DriveForwardBasicCommand(DISTANCE_TO_GET_OVER_BASELINE), new EnableMotionProfile(DISTANCE_TO_GET_OVER_BASELINE, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {
			
			@Override
			protected boolean condition() {
				return AutoChoosers.chosen == AutoChoosers.ProfilingMethod.basic;
			}
		});

	}
}
