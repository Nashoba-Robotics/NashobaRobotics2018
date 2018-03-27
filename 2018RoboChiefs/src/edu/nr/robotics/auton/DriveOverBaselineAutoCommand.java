package edu.nr.robotics.auton;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveForwardBasicCommand;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class DriveOverBaselineAutoCommand extends CommandGroup {
	
	public DriveOverBaselineAutoCommand() {

		
		addSequential(new ConditionalCommand(new DriveForwardBasicCommand(FieldMeasurements.BASELINE_TO_MID_SWITCH_X), new EnableMotionProfile(FieldMeasurements.BASELINE_TO_MID_SWITCH_X, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {
			
			@Override
			protected boolean condition() {
				return AutoChoosers.chosen == AutoChoosers.ProfilingMethod.basic;
			}
		});

	}
}
