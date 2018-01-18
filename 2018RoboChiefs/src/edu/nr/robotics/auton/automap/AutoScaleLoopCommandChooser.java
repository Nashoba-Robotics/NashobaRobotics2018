package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.AllianceBlocks;
import edu.nr.robotics.auton.AutoChoosers.Switch;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class AutoScaleLoopCommandChooser extends CommandGroup {

	public AutoScaleLoopCommandChooser() {
		addSequential(new ConditionalCommand(new AutoScaleLoopCommand(2)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.left
						&& (((Robot.getInstance().selectedSwitch == Switch.leftOnly
								|| Robot.getInstance().selectedSwitch == Switch.both)
								&& FieldData.getInstance().nearSwitch == Direction.left)
								|| (Robot.getInstance().selectedBlocks == AllianceBlocks.block1
										|| Robot.getInstance().selectedBlocks == AllianceBlocks.both));
			}

		});

		addSequential(new ConditionalCommand(new AutoScaleLoopCommand(1)) {

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

		addSequential(new ConditionalCommand(new AutoScaleLoopCommand(5)) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().scale == Direction.right
						&& (((Robot.getInstance().selectedSwitch == Switch.rightOnly
								|| Robot.getInstance().selectedSwitch == Switch.both)
								&& FieldData.getInstance().nearSwitch == Direction.right)
								|| (Robot.getInstance().selectedBlocks == AllianceBlocks.block6
										|| Robot.getInstance().selectedBlocks == AllianceBlocks.both));
			}

		});

		addSequential(new ConditionalCommand(new AutoScaleLoopCommand(6)) {

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
	}

}
