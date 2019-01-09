package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterShootCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosMiddleToSwitchRightProfilingCommand extends CommandGroup {

	public StartPosMiddleToSwitchRightProfilingCommand() {
		
		addSequential(new EnableMotionProfile(FieldMeasurements.BASELINE_TO_SWITCH_X, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
					
		addSequential(new ElevatorProfileCommandGroup(Elevator.SWITCH_HEIGHT_ELEVATOR, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
		
		addSequential(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_HIGH_ELEVATOR_SHOOTER));
		
	}
	
}
