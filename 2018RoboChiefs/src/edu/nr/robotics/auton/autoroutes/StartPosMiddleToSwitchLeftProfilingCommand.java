package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveCurrentCommand;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosMiddleToSwitchLeftProfilingCommand extends CommandGroup {

	public StartPosMiddleToSwitchLeftProfilingCommand() {
		
		addSequential(new EnableMotionProfile(FieldMeasurements.PUSH_OFF_WALL_X, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnCommand(Drive.getInstance(), (new Angle(90, Angle.Unit.DEGREE).sub(FieldMeasurements.START_POS_MID_TO_SWITCH_ANGLE_LEFT)).negate(), 
				Drive.MAX_PROFILE_TURN_PERCENT, true));
		
		addSequential(new EnableMotionProfile(FieldMeasurements.START_POS_MID_TO_SWITCH_LEFT_DIAGONAL, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnCommand(Drive.getInstance(), new Angle(180, Angle.Unit.DEGREE).sub(FieldMeasurements.START_POS_MID_TO_SWITCH_ANGLE_LEFT), 
				Drive.MAX_PROFILE_TURN_PERCENT, true));
		
		addSequential(new DriveCurrentCommand(Drive.SWITCH_DRIVE_PERCENT, Drive.SWITCH_CURRENT_LIMIT));

	}
	
}
