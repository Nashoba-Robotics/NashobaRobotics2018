package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.StartAutoCommand;
import edu.nr.robotics.auton.autoroutes.BlockLeftToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.BlockLeftToScaleRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.BlockRightToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.BlockRightToScaleRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToSwitchLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToSwitchRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.SwitchLeftToBlockProfilingCommand;
import edu.nr.robotics.auton.autoroutes.SwitchRightToBlockProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosLeftSwitchBothCommand extends CommandGroup {
	
	public StartPosLeftSwitchBothCommand() {
		
		addSequential(new ConditionalCommand(new StartPosLeftToSwitchLeftProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
			
		});
		
		addSequential(new ConditionalCommand(new SwitchLeftToBlockProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left && StartAutoCommand.selectedScale == Scale.yes;
			}
			
		});
		
		addSequential(new ConditionalCommand(new StartPosLeftToSwitchRightProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right;
			}
			
		});
		
		addSequential(new ConditionalCommand(new SwitchRightToBlockProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right && StartAutoCommand.selectedScale == Scale.yes;
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
		
		addSequential(new ConditionalCommand(new BlockRightToScaleLeftProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right && StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.left;
			}
			
		});
		
		addSequential(new ConditionalCommand(new BlockRightToScaleRightProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right && StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.right;
			}
			
		});
		
	}

}
