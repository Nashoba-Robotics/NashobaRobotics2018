package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveCurrentCommand;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnPIDCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosLeftToSwitchLeftProfilingCommand extends CommandGroup {
	
	public StartPosLeftToSwitchLeftProfilingCommand() {
	
		addSequential(new EnableMotionProfile(FieldMeasurements.BASELINE_TO_SWITCH_X, FieldMeasurements.BASELINE_TO_SWITCH_Y, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
		
		addSequential(new DriveCurrentCommand(Drive.SWITCH_DRIVE_PERCENT, Drive.SWITCH_CURRENT_LIMIT));
	}

}
