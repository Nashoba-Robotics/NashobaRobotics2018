package edu.nr.robotics.auton.autoroutes;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.multicommands.PrepareScoreSwitchAutoCommand;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterShootCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class Cube1ToSwitchProfilingCommand extends CommandGroup {
	
	public Cube1ToSwitchProfilingCommand() {
		
		addSequential(new PrepareScoreSwitchAutoCommand());
		
		addSequential(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				addParallel(new EnableMotionProfile(FieldMeasurements.CUBE_1_TO_SWITCH_DIAGONAL, Distance.ZERO,
						Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
				
				addParallel(new ElevatorProfileCommandGroup(Elevator.SWITCH_HEIGHT_ELEVATOR,
						Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
				
			}
		});
		
		addSequential(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_HIGH_ELEVATOR_SHOOTER));
		
	}

}
