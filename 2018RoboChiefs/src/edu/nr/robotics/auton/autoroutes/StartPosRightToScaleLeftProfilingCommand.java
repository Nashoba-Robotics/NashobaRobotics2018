package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterShootCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeDeployCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class StartPosRightToScaleLeftProfilingCommand extends CommandGroup {

public StartPosRightToScaleLeftProfilingCommand() {
		
		addSequential(new EnableMotionProfile(FieldMeasurements.BASELINE_TO_PLATFORM_ZONE_X, FieldMeasurements.BASELINE_TO_PLATFORM_ZONE_Y, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
				
		addSequential(new TurnCommand(Drive.getInstance(), new Angle(-90,Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));
			
		addParallel(new IntakeDeployCommand());
		
		addSequential(new EnableMotionProfile(FieldMeasurements.PLATFORM_ZONE_WIDTH_START_POS, Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new TurnCommand(Drive.getInstance(), new Angle(180, Angle.Unit.DEGREE).sub(FieldMeasurements.PIVOT_POINT_TO_SCALE_ACROSS_FIELD), Drive.MAX_PROFILE_TURN_PERCENT));
		
		addSequential(new EnableMotionProfile(FieldMeasurements.PLATFORM_ZONE_TO_SCALE_DIAGONAL.sub(new Distance(6, Distance.Unit.INCH)), Distance.ZERO, Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new ElevatorProfileCommandGroup(Elevator.SCALE_HEIGHT_ELEVATOR, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));	
		
		addSequential(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_SCALE_AUTO_ELEVATOR_SHOOTER));
					
	}
	
}
