package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.multicommands.DriveToCubeCommandAdvanced;;

public class SwitchRightToBlockProfilingCommand extends CommandGroup {

	public SwitchRightToBlockProfilingCommand() {

		addSequential(new EnableMotionProfile(
				(FieldMeasurements.SWITCH_EDGE_TO_FIELD_EDGE_Y.sub(FieldMeasurements.PIVOT_POINT_FIELD_EDGE_Y)
						.sub(FieldMeasurements.ROBOT_LENGTH.mul(0.5))).negate(),
				Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));

		addSequential(new TurnCommand(Drive.getInstance(), new Angle(-90, Angle.Unit.DEGREE),
				Drive.MAX_PROFILE_TURN_PERCENT, true));

		addSequential(new EnableMotionProfile(FieldMeasurements.SWITCH_TO_PIVOT_POINT_X.negate(), Distance.ZERO,
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new EnableLimelightCommand(true));

		addSequential(new TurnCommand(Drive.getInstance(),
				(new Angle(90, Angle.Unit.DEGREE).sub(FieldMeasurements.PIVOT_POINT_TO_CUBE_1)),
				Drive.MAX_PROFILE_TURN_PERCENT, true));

		addSequential(new DriveToCubeCommandAdvanced());
		
		addSequential(new EnableLimelightCommand(false));
	}

}
