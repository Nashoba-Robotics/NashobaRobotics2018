package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldDistances;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnPIDCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosFarRightToSwitchRightProfilingCommand extends CommandGroup {

	public StartPosFarRightToSwitchRightProfilingCommand() {
		
addSequential(new EnableMotionProfile(FieldDistances.BASELINE_TO_SWITCH_X, FieldDistances.BASELINE_TO_SWITCH_Y, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(-90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, true));
	}
	
}
