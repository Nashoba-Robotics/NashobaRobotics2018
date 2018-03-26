package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.AutoChoosers;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.autoroutes.BlockToScaleProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToScaleRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToSwitchLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.PivotSwitchLeftToBlockProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class StartPosMiddleSwitchLeftCommand extends CommandGroup {

	public StartPosMiddleSwitchLeftCommand() {

		addSequential(new WaitCommand(Robot.getInstance().autoWaitTime));

		addSequential(new ConditionalCommand(new StartPosMiddleToSwitchLeftProfilingCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
		});

		addSequential(new ConditionalCommand(new PivotSwitchLeftToBlockProfilingCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left
						&& (Robot.getInstance().selectedScale == Scale.both
								|| (Robot.getInstance().selectedScale == Scale.leftonly
										&& FieldData.getInstance().scale == Direction.left)
								|| (Robot.getInstance().selectedScale == Scale.rightonly
										&& FieldData.getInstance().scale == Direction.right));
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(1)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left
						&& (Robot.getInstance().selectedScale == Scale.both
								|| Robot.getInstance().selectedScale == Scale.leftonly)
						&& FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(1)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left
						&& (Robot.getInstance().selectedScale == Scale.both
								|| Robot.getInstance().selectedScale == Scale.rightonly)
						&& FieldData.getInstance().scale == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new StartPosMiddleToScaleLeftProfilingCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right
						&& (Robot.getInstance().selectedScale == Scale.both
								|| Robot.getInstance().selectedScale == Scale.leftonly)
						&& FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new StartPosMiddleToScaleRightProfilingCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right
						&& (Robot.getInstance().selectedScale == Scale.both
								|| Robot.getInstance().selectedScale == Scale.rightonly)
						&& FieldData.getInstance().scale == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new DriveOverBaselineAutoCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right
						&& (Robot.getInstance().selectedScale == Scale.none
								|| (Robot.getInstance().selectedScale == Scale.leftonly
										&& FieldData.getInstance().scale == Direction.right)
								|| (Robot.getInstance().selectedScale == Scale.rightonly
										&& FieldData.getInstance().scale == Direction.left));
			}

		});

		addSequential(new ConditionalCommand(new AutoScaleLoopCommand()) {

			@Override
			protected boolean condition() {
				return Robot.getInstance().selectedScale == Scale.both
						|| (Robot.getInstance().selectedScale == Scale.leftonly
								&& FieldData.getInstance().scale == Direction.left)
						|| (Robot.getInstance().selectedScale == Scale.rightonly
								&& FieldData.getInstance().scale == Direction.right);
			}

		});
		
		addSequential(new ConditionalCommand(new AutoSwitchLoopCommand()) {

			@Override
			protected boolean condition() {
				return ((Robot.getInstance().selectedSwitch == AutoChoosers.Switch.leftOnly
						|| Robot.getInstance().selectedSwitch == AutoChoosers.Switch.both)
						&& FieldData.getInstance().nearSwitch == Direction.left)
						&& !(Robot.getInstance().selectedScale == Scale.both
								|| (Robot.getInstance().selectedScale == Scale.leftonly
										&& FieldData.getInstance().scale == Direction.left)
								|| (Robot.getInstance().selectedScale == Scale.rightonly
										&& FieldData.getInstance().scale == Direction.right));
			}

		});


	}

}
