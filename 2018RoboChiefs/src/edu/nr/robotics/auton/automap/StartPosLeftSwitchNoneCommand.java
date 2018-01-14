package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.StartAutoCommand;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToScaleRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToSwitchLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToSwitchRightProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosLeftSwitchNoneCommand extends CommandGroup {
	
	public StartPosLeftSwitchNoneCommand() {
		
		addSequential(new ConditionalCommand(new StartPosLeftToScaleLeftProfilingCommand()){

			@Override
			protected boolean condition() {
				return StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.left;
			}
			
		});
		
		addSequential(new ConditionalCommand(new StartPosLeftToScaleRightProfilingCommand()){

			@Override
			protected boolean condition() {
				return StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.right;
			}
			
		});
		
		addSequential(new ConditionalCommand(new DriveOverBaselineAutoCommand()){

			@Override
			protected boolean condition() {
				return StartAutoCommand.selectedScale == Scale.no;
			}
			
		});
		
	}

}
