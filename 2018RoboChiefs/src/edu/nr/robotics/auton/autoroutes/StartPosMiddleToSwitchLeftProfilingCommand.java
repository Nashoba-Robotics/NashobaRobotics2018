package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveCurrentCommand;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterShootCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosMiddleToSwitchLeftProfilingCommand extends CommandGroup {

	public StartPosMiddleToSwitchLeftProfilingCommand() {

		addSequential(new EnableMotionProfile(FieldMeasurements.PUSH_OFF_WALL_X, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnCommand(Drive.getInstance(), FieldMeasurements.START_POS_MID_TO_SWITCH_ANGLE_LEFT.negate(), Drive.MAX_PROFILE_TURN_PERCENT));
		
		addSequential(new EnableMotionProfile(FieldMeasurements.START_POS_MID_TO_SWITCH_LEFT_DIAGONAL, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));

		addSequential(new TurnCommand(Drive.getInstance(), FieldMeasurements.START_POS_MID_TO_SWITCH_ANGLE_LEFT, Drive.MAX_PROFILE_TURN_PERCENT));
	
		addSequential(new EnableMotionProfile(FieldMeasurements.BASELINE_TO_SWITCH_X.sub(FieldMeasurements.BASELINE_TO_10_CUBE_PILE).add(FieldMeasurements.ROBOT_LENGTH), Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
				
		addSequential(new ElevatorProfileCommandGroup(Elevator.SWITCH_HEIGHT_ELEVATOR, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
		
		addSequential(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_HIGH_ELEVATOR_SHOOTER));
		
	}
	
}
