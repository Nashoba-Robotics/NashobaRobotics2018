package edu.nr.robotics.auton.automap;

import edu.nr.lib.units.Distance;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.auton.autoroutes.PlatformZoneSwitchLeftToBlockProfilingCommand;
import edu.nr.robotics.auton.autoroutes.PlatformZoneSwitchRightToBlockProfilingCommand;
import edu.nr.robotics.multicommands.DriveToCubeCommandAdvanced;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveCurrentCommand;
import edu.nr.robotics.subsystems.drive.EnableMotionProfile;
import edu.nr.robotics.subsystems.drive.StrafeToCubeCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class AutoSwitchLoopCommand extends CommandGroup {

	public AutoSwitchLoopCommand() {
		
		addSequential(new ConditionalCommand(new PlatformZoneSwitchLeftToBlockProfilingCommand(), new PlatformZoneSwitchRightToBlockProfilingCommand()) {
			
			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});

		addSequential(new ConditionalCommand(new EnableMotionProfile(Distance.ZERO, FieldMeasurements.CUBE_1_AND_2_TO_SWITCH_Y.negate(), 
						Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT),
				new EnableMotionProfile(Distance.ZERO, FieldMeasurements.CUBE_1_AND_2_TO_SWITCH_Y, 
						Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {

			@Override
			protected boolean condition() {				
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});
		
		addSequential(new DriveCurrentCommand(Drive.SWITCH_DRIVE_PERCENT, Drive.SWITCH_CURRENT_LIMIT));
		
		addSequential(new EnableMotionProfile(FieldMeasurements.CUBE_TO_PLATFORM_ZONE_X.negate(), Distance.ZERO,
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new ConditionalCommand(new StrafeToCubeCommand(Direction.left), new StrafeToCubeCommand(Direction.right)) {
			
			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});
		
		addSequential(new DriveToCubeCommandAdvanced());
		
		addSequential(new ConditionalCommand(new EnableMotionProfile(Distance.ZERO, FieldMeasurements.CUBE_1_AND_2_TO_SWITCH_Y, 
					Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT),
			new EnableMotionProfile(Distance.ZERO, FieldMeasurements.CUBE_1_AND_2_TO_SWITCH_Y.negate(), 
					Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {

			@Override
			protected boolean condition() {				
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});

		addSequential(new DriveCurrentCommand(Drive.SWITCH_DRIVE_PERCENT, Drive.SWITCH_CURRENT_LIMIT));
		
		addSequential(new EnableMotionProfile(FieldMeasurements.CUBE_TO_PLATFORM_ZONE_X.negate(), Distance.ZERO,
				Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT));
		
		addSequential(new ConditionalCommand(new StrafeToCubeCommand(Direction.left), new StrafeToCubeCommand(Direction.right)) {
			
			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});
		
		addSequential(new DriveToCubeCommandAdvanced());
		
		addSequential(new ConditionalCommand(new EnableMotionProfile(Distance.ZERO, FieldMeasurements.CUBE_3_TO_SWITCH_Y, 
					Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT),
			new EnableMotionProfile(Distance.ZERO, FieldMeasurements.CUBE_3_TO_SWITCH_Y.negate(), 
					Drive.PROFILE_DRIVE_PERCENT, Drive.ACCEL_PERCENT)) {
	
			@Override
			protected boolean condition() {				
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});
	}
	
}
