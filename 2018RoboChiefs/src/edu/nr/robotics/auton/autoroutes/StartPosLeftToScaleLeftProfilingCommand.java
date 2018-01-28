package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.AutoChoosers;
import edu.nr.robotics.auton.AutoChoosers.ProfilingMethod;
import edu.nr.robotics.auton.FieldDistances;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveForwardBasicCommand;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnPIDCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class StartPosLeftToScaleLeftProfilingCommand extends CommandGroup {

	public StartPosLeftToScaleLeftProfilingCommand() {
		
		addSequential(new EnableMotionProfile(FieldDistances.BASELINE_TO_SCALE_X, FieldDistances.BASELINE_TO_SCALE_Y, 
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
						
		addSequential(new WaitCommand(0));
		
		addSequential(new TurnPIDCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, false));
		
	}
	
}
