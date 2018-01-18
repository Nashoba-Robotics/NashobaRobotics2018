package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.autoroutes.StartPosFarRightToScaleLeftProfilingCommand;
import edu.nr.robotics.auton.autoroutes.StartPosFarRightToScaleRightProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StartPosFarRightSwitchNoneCommand extends CommandGroup {

	public StartPosFarRightSwitchNoneCommand() {

		addSequential(new ConditionalCommand(new StartPosFarRightToScaleRightProfilingCommand()) {

			@Override
			protected boolean condition() {
				return Robot.getInstance().selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.right;
			}

		});

		addSequential(new ConditionalCommand(new StartPosFarRightToScaleLeftProfilingCommand()) {

			@Override
			protected boolean condition() {
				return Robot.getInstance().selectedScale == Scale.yes && FieldData.getInstance().scale == Direction.left;
			}

		});

		addSequential(new ConditionalCommand(new DriveOverBaselineAutoCommand()) {

			@Override
			protected boolean condition() {
				return Robot.getInstance().selectedScale == Scale.no;
			}

		});
		
		addSequential(new ConditionalCommand(new AutoScaleLoopCommandChooser()){

			@Override
			protected boolean condition() {
				return Robot.getInstance().selectedScale == Scale.yes;
			}
			
		});

	}

}
