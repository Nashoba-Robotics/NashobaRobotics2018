package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.StartAutoCommand;
import edu.nr.robotics.auton.autoroutes.BlockLeftToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.BlockLeftToScaleRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToScaleRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToSwitchLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.SwitchLeftToBlockProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosMiddleSwitchLeftCommand extends CommandGroup {
	
	public StartPosMiddleSwitchLeftCommand() {
		
		//Checks
		addSequential(new ConditionalCommand(new StartPosMiddleToSwitchLeftProfilingCommand()) {
			
			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
		});
		
		addSequential(new ConditionalCommand(new SwitchLeftToBlockProfilingCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left && StartAutoCommand.selectedScale == Scale.yes;
			}
			
		});
		
		addSequential(new ConditionalCommand(new BlockLeftToScaleLeftProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left && StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.left;
			}
			
		});
		
		addSequential(new ConditionalCommand(new BlockLeftToScaleRightProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left && StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.right;
			}
			
		});
		
		addSequential(new ConditionalCommand(new StartPosMiddleToScaleLeftProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right && StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.left;
			}
			
		});
		
		addSequential(new ConditionalCommand(new StartPosMiddleToScaleRightProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right && StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.right;
			}
			
		});
		
		addSequential(new ConditionalCommand(new DriveOverBaselineAutoCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right && StartAutoCommand.selectedScale == Scale.no;
			}
			
		});
		
		
	}

}
