package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.StartAutoCommand;
import edu.nr.robotics.auton.autoroutes.BlockToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.BlockToScaleRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosFarRightToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosFarRightToScaleRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosFarRightToSwitchRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.SwitchRightToBlockProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosFarRightSwitchRightCommand extends CommandGroup {
	
	public StartPosFarRightSwitchRightCommand() {
		
		addSequential(new ConditionalCommand(new StartPosFarRightToSwitchRightProfilingCommand()){
			
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
		
		addSequential(new ConditionalCommand(new BlockToScaleLeftProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right && StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.left;
			}
			
		});
		
		addSequential(new ConditionalCommand(new BlockToScaleRightProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right && StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.right;
			}
			
		});
		
		addSequential(new ConditionalCommand(new StartPosFarRightToScaleRightProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left && StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.right;
			}
			
		});
		
		addSequential(new ConditionalCommand(new StartPosFarRightToScaleLeftProfilingCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left && StartAutoCommand.selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.left;
			}
			
		});
		
		addSequential(new ConditionalCommand(new DriveOverBaselineAutoCommand()){

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left && StartAutoCommand.selectedScale == Scale.no;
			}
			
		});
	}

}
