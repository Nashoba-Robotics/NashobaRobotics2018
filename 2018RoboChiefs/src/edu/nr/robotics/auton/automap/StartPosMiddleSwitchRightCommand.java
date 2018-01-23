package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.autoroutes.BlockToScaleProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToScaleRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToSwitchRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.SwitchRightToBlockProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class StartPosMiddleSwitchRightCommand extends CommandGroup {

	public StartPosMiddleSwitchRightCommand() {
		
		addSequential(new WaitCommand(Robot.getInstance().autoWaitTime));

		addSequential(new ConditionalCommand(new StartPosMiddleToSwitchRightProfilingCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new SwitchRightToBlockProfilingCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right
						&& Robot.getInstance().selectedScale == Scale.yes;
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(6)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right
						&& Robot.getInstance().selectedScale == Scale.yes
						&& FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(6)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right
						&& Robot.getInstance().selectedScale == Scale.yes
						&& FieldData.getInstance().scale == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new StartPosMiddleToScaleLeftProfilingCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left
						&& Robot.getInstance().selectedScale == Scale.yes
						&& FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new StartPosMiddleToScaleRightProfilingCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left
						&& Robot.getInstance().selectedScale == Scale.yes
						&& FieldData.getInstance().scale == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new DriveOverBaselineAutoCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left
						&& Robot.getInstance().selectedScale == Scale.no;
			}

		});

		addSequential(new ConditionalCommand(new AutoScaleLoopCommand()){

			@Override
			protected boolean condition() {
				return Robot.getInstance().selectedScale == Scale.yes;
			}
			
		});
	}

}
