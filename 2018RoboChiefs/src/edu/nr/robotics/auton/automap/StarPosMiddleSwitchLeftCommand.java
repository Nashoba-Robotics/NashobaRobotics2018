package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.StartAutoCommand;
import edu.nr.robotics.auton.autoroutes.BlockToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.BlockToScaleRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToLeftSwitchProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToScaleRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.SwitchLeftToBlockProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StarPosMiddleSwitchLeftCommand extends CommandGroup {
	
	public StarPosMiddleSwitchLeftCommand() {
		
		//Checks
		addSequential(new ConditionalCommand(new StartPosMiddleToLeftSwitchProfilingCommand()) {
			
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
		
		addSequential(new ConditionalCommand(new BlockToScaleLeftProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left && StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.left;
			}
			
		});
		
		addSequential(new ConditionalCommand(new BlockToScaleRightProfilingCommand()){

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
