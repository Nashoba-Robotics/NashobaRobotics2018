package edu.nr.robotics.auton.autoroutes;


import edu.nr.lib.units.Distance;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.multicommands.DriveToCubeCommandAdvanced;
import edu.nr.robotics.multicommands.DriveToCubeCommandBasic;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.TurnPIDCommand;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class ScaleToBlockProfilingCommand extends CommandGroup {
	
	public ScaleToBlockProfilingCommand(int block) {
		
		addSequential(new EnableMotionProfile(FieldMeasurements.PLATFORM_ZONE_TO_SCALE_DIAGONAL.negate(), Distance.ZERO, 
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new EnableLimelightCommand(true));
		
		//Turn to right location
		addSequential(new ConditionalCommand(new TurnPIDCommand(Drive.getInstance(),
				(FieldMeasurements.PLATFORM_ZONE_TO_SCALE.add(FieldMeasurements.PLATFORM_ZONE_TO_CUBE)),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 1;
			}
			
		});
		
		addSequential(new ConditionalCommand(new TurnPIDCommand(Drive.getInstance(),
				(FieldMeasurements.PLATFORM_ZONE_TO_SCALE.add(FieldMeasurements.PLATFORM_ZONE_TO_CUBE)).negate(),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 6;
			}
			
		});
		
		addSequential(new ConditionalCommand(new TurnPIDCommand(Drive.getInstance(),
				FieldMeasurements.PLATFORM_ZONE_TO_SCALE,
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 2 || block == 3;
			}
			
		});
		
		addSequential(new ConditionalCommand(new TurnPIDCommand(Drive.getInstance(),
				FieldMeasurements.PLATFORM_ZONE_TO_SCALE.negate(),
				Drive.MAX_PROFILE_TURN_PERCENT, true)) {

			@Override
			protected boolean condition() {
				return block == 4 || block == 5;
			}
			
		});
		
		//TODO: Add in distances to inner blocks
		
		addSequential(new DriveToCubeCommandAdvanced());
		
		addSequential(new EnableLimelightCommand(false));
		
	}

}
