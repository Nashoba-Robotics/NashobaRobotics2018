package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveCurrentCommand;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosFarRightToSwitchRightProfilingCommand extends CommandGroup {

	public StartPosFarRightToSwitchRightProfilingCommand() {
		
addSequential(new EnableMotionProfile(FieldMeasurements.BASELINE_TO_SWITCH_X, FieldMeasurements.BASELINE_TO_SWITCH_Y, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnCommand(Drive.getInstance(), new Angle(-90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
	
		addSequential(new DriveCurrentCommand(Drive.SWITCH_DRIVE_PERCENT, Drive.SWITCH_CURRENT_LIMIT));
	}
	
}
