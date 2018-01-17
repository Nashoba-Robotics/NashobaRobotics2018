package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.Robot;
import edu.nr.robotics.FieldData.Direction;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class AutoScaleLoopCommand extends CommandGroup {

	private double block1;
	private double block2;

	public AutoScaleLoopCommand(double block1, double block2) {
		this.block1 = block1;
		this.block2 = block2;

	}

	public AutoScaleLoopCommand() {
		this(0, 0);
	}

}
