package edu.nr.robotics.auton;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.Robot;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveForwardBasicCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveOverBaselineAutoCommand extends CommandGroup {

	/**
	 * The distance (in inches) to drive to get over baseline safely in auto.
	 * 
	 * It needs to be at least 62 inches, but more is safer.
	 */
	public static final Distance DISTANCE_TO_GET_OVER_BASELINE = new Distance(0, Distance.Unit.INCH); //TODO: Put in distance to baseline
	
	/**
	 * The speed in percent to drive forward to get over baseline safely in auto
	 */
	public static final double FORWARD_PERCENT = 0.25;
	
	public DriveOverBaselineAutoCommand() {

		addSequential(new DriveForwardBasicCommand(Drive.PROFILE_DRIVE_PERCENT, DISTANCE_TO_GET_OVER_BASELINE));

	}
}
