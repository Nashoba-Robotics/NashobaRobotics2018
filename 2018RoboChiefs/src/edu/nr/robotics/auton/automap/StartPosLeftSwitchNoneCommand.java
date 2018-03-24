package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosLeftToScaleRightProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class StartPosLeftSwitchNoneCommand extends CommandGroup {

	public StartPosLeftSwitchNoneCommand() {

		addSequential(new WaitCommand(Robot.getInstance().autoWaitTime));

		addSequential(new ConditionalCommand(new StartPosLeftToScaleLeftProfilingCommand()) {

			@Override
			protected boolean condition() {
				return (Robot.getInstance().selectedScale == Scale.both
						|| Robot.getInstance().selectedScale == Scale.leftonly)
						&& FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new StartPosLeftToScaleRightProfilingCommand()) {

			@Override
			protected boolean condition() {
				return (Robot.getInstance().selectedScale == Scale.both
						|| Robot.getInstance().selectedScale == Scale.rightonly)
						&& FieldData.getInstance().scale == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new DriveOverBaselineAutoCommand()) {

			@Override
			protected boolean condition() {
				return Robot.getInstance().selectedScale == Scale.none
						|| (Robot.getInstance().selectedScale == Scale.leftonly
								&& FieldData.getInstance().scale == Direction.right)
						|| (Robot.getInstance().selectedScale == Scale.rightonly
								&& FieldData.getInstance().scale == Direction.left);
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
	}

}
