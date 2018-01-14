package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.StartAutoCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToScaleRightProfilingCommand;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosMiddleSwitchNoneCommand extends CommandGroup {

	public StartPosMiddleSwitchNoneCommand() {

		addSequential(new ConditionalCommand(new StartPosMiddleToScaleLeftProfilingCommand()) {

			@Override
			protected boolean condition() {
				return StartAutoCommand.selectedScale == Scale.yes 
						&& FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new StartPosMiddleToScaleRightProfilingCommand()) {

			@Override
			protected boolean condition() {
				return StartAutoCommand.selectedScale == Scale.yes 
						&& FieldData.getInstance().scale == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new DriveOverBaselineAutoCommand()) {

			@Override
			protected boolean condition() {
				return StartAutoCommand.selectedScale == Scale.no;
			}

		});

	}

}
