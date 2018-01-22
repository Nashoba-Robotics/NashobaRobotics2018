package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.AllianceBlocks;
import edu.nr.robotics.auton.AutoChoosers.Switch;
import edu.nr.robotics.auton.autoroutes.BlockToScaleProfilingCommand;
import edu.nr.robotics.auton.autoroutes.ScaleToBlockProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class AutoScaleLoopCommand extends CommandGroup {

	public AutoScaleLoopCommand() {
		addSequential(new ConditionalCommand(new ScaleToBlockProfilingCommand(1)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.left
						&& !(((Robot.getInstance().selectedSwitch == Switch.leftOnly
								|| Robot.getInstance().selectedSwitch == Switch.both)
								&& FieldData.getInstance().nearSwitch == Direction.left)
								|| (Robot.getInstance().selectedBlocks == AllianceBlocks.block1
										|| Robot.getInstance().selectedBlocks == AllianceBlocks.both));
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(1)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.left
						&& !(((Robot.getInstance().selectedSwitch == Switch.leftOnly
								|| Robot.getInstance().selectedSwitch == Switch.both)
								&& FieldData.getInstance().nearSwitch == Direction.left)
								|| (Robot.getInstance().selectedBlocks == AllianceBlocks.block1
										|| Robot.getInstance().selectedBlocks == AllianceBlocks.both));
			}

		});

		addSequential(new ConditionalCommand(new ScaleToBlockProfilingCommand(2)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(2)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new ScaleToBlockProfilingCommand(3)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(3)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new ScaleToBlockProfilingCommand(6)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.right
						&& !(((Robot.getInstance().selectedSwitch == Switch.rightOnly
								|| Robot.getInstance().selectedSwitch == Switch.both)
								&& FieldData.getInstance().nearSwitch == Direction.right)
								|| (Robot.getInstance().selectedBlocks == AllianceBlocks.block6
										|| Robot.getInstance().selectedBlocks == AllianceBlocks.both));
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(6)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.right
						&& !(((Robot.getInstance().selectedSwitch == Switch.rightOnly
								|| Robot.getInstance().selectedSwitch == Switch.both)
								&& FieldData.getInstance().nearSwitch == Direction.right)
								|| (Robot.getInstance().selectedBlocks == AllianceBlocks.block6
										|| Robot.getInstance().selectedBlocks == AllianceBlocks.both));
			}

		});

		addSequential(new ConditionalCommand(new ScaleToBlockProfilingCommand(5)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(5)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new ScaleToBlockProfilingCommand(4)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new BlockToScaleProfilingCommand(4)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.right;
			}

		});
	}

}
