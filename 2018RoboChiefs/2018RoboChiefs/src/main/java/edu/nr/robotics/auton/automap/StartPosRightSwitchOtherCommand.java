package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.DriveOverBaselineFancyCommand;
import edu.nr.robotics.auton.autoroutes.StartPosRightToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosRightToScaleRightProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosRightToSwitchRightProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class StartPosRightSwitchOtherCommand extends CommandGroup {

	public StartPosRightSwitchOtherCommand() {

		addSequential(new WaitCommand(Robot.getInstance().autoWaitTime));

		addSequential(new ConditionalCommand(new StartPosRightToScaleRightProfilingCommand()) {

			@Override
			protected boolean condition() {
				return (Robot.getInstance().selectedScale == Scale.both
						|| Robot.getInstance().selectedScale == Scale.rightonly)
						&& FieldData.getInstance().scale == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new StartPosRightToScaleLeftProfilingCommand()) {

			@Override
			protected boolean condition() {
				return (Robot.getInstance().selectedScale == Scale.both
						|| Robot.getInstance().selectedScale == Scale.leftonly)
						&& FieldData.getInstance().scale == Direction.left;
			}

		});
		
		addSequential(new ConditionalCommand(new StartPosRightToSwitchRightProfilingCommand()) {

			@Override
			protected boolean condition() {
				return !(Robot.getInstance().selectedScale == Scale.both
						|| (Robot.getInstance().selectedScale == Scale.rightonly
								&& FieldData.getInstance().scale == Direction.right)
						|| (Robot.getInstance().selectedScale == Scale.leftonly
								|| FieldData.getInstance().scale == Direction.left))
						&& FieldData.getInstance().nearSwitch == Direction.right;
			}
			
		});
		
		addSequential(new ConditionalCommand(new DriveOverBaselineFancyCommand()) {

			@Override
			protected boolean condition() {
				return !(Robot.getInstance().selectedScale == Scale.both
						|| (Robot.getInstance().selectedScale == Scale.rightonly
								&& FieldData.getInstance().scale == Direction.right)
						|| (Robot.getInstance().selectedScale == Scale.leftonly
								|| FieldData.getInstance().scale == Direction.left))
						&& FieldData.getInstance().nearSwitch == Direction.left
						&& FieldData.getInstance().scale == Direction.left;
			}
			
		});

		addSequential(new ConditionalCommand(new DriveOverBaselineAutoCommand()) {

			@Override
			protected boolean condition() {
				return !(Robot.getInstance().selectedScale == Scale.both
						|| (Robot.getInstance().selectedScale == Scale.rightonly
						&& FieldData.getInstance().scale == Direction.right)
				|| (Robot.getInstance().selectedScale == Scale.leftonly
						|| FieldData.getInstance().scale == Direction.left))
				&& FieldData.getInstance().nearSwitch == Direction.left
				&& FieldData.getInstance().scale == Direction.right;
			}

		});
		
		addSequential(new ConditionalCommand(new AutoScaleLoopCommand()) {

			@Override
			protected boolean condition() {
				return Robot.getInstance().selectedScale == Scale.both
						|| (Robot.getInstance().selectedScale == Scale.rightonly
								&& FieldData.getInstance().scale == Direction.right)
						|| (Robot.getInstance().selectedScale == Scale.leftonly
								|| FieldData.getInstance().scale == Direction.left);
			}

		});

	}

}
