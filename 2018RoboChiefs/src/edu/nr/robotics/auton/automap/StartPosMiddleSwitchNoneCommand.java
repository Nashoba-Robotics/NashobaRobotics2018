package edu.nr.robotics.auton.automap;

import edu.nr.robotics.Robot;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class StartPosMiddleSwitchNoneCommand extends CommandGroup {

	public StartPosMiddleSwitchNoneCommand() {

		addSequential(new WaitCommand(Robot.getInstance().autoWaitTime));

		addSequential(new DriveOverBaselineAutoCommand());

	}

}
