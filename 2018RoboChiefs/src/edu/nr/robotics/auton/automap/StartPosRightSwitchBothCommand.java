package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.autoroutes.BlockToScaleProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosRightToSwitchLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosRightToSwitchRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.SwitchLeftToBlockProfilingCommand;
import edu.nr.robotics.auton.autoroutes.SwitchRightToBlockProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class StartPosRightSwitchBothCommand extends CommandGroup {

	public StartPosRightSwitchBothCommand() {

		addSequential(new WaitCommand(Robot.getInstance().autoWaitTime));

		addSequential(new ConditionalCommand(new StartPosRightToSwitchLeftProfilingCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new SwitchLeftToBlockProfilingCommand()) {

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

		addSequential(new ConditionalCommand(new StartPosRightToSwitchRightProfilingCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new SwitchRightToBlockProfilingCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right
						&& (Robot.getInstance().selectedScale == Scale.both
								|| (Robot.getInstance().selectedScale == Scale.leftonly
										&& FieldData.getInstance().scale == Direction.left)
								|| (Robot.getInstance().selectedScale == Scale.rightonly
										&& FieldData.getInstance().scale == Direction.right));
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(6)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right
						&& (Robot.getInstance().selectedScale == Scale.both
								|| Robot.getInstance().selectedScale == Scale.leftonly)
						&& FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(6)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.right
						&& (Robot.getInstance().selectedScale == Scale.both
								|| Robot.getInstance().selectedScale == Scale.rightonly)
						&& FieldData.getInstance().scale == Direction.right;
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
				return ((Robot.getInstance().selectedSwitch == AutoChoosers.Switch.rightOnly
						|| Robot.getInstance().selectedSwitch == AutoChoosers.Switch.both)
						&& FieldData.getInstance().nearSwitch == Direction.right)
						&& !(Robot.getInstance().selectedScale == Scale.both
								|| (Robot.getInstance().selectedScale == Scale.leftonly
										&& FieldData.getInstance().scale == Direction.left)
								|| (Robot.getInstance().selectedScale == Scale.rightonly
										&& FieldData.getInstance().scale == Direction.right));
			}
			
		});

	}

}
