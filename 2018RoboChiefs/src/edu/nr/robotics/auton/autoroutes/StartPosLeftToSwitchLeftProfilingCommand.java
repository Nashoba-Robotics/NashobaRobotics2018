package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
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

public class StartPosLeftToSwitchLeftProfilingCommand extends CommandGroup {
	
	public StartPosLeftToSwitchLeftProfilingCommand() {
	
		addSequential(new EnableMotionProfile(FieldMeasurements.BASELINE_TO_MID_SWITCH_X, FieldMeasurements.BASELINE_TO_SWITCH_Y, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));
		
		addSequential(new DriveCurrentCommand(Drive.SWITCH_DRIVE_PERCENT, Drive.SWITCH_CURRENT_LIMIT));
		
		addSequential(new ElevatorProfileCommandGroup(Elevator.SWITCH_HEIGHT_ELEVATOR, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
				
		addSequential(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_HIGH_ELEVATOR_SHOOTER));
		
	}
}
