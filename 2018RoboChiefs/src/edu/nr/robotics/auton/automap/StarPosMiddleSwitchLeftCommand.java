package edu.nr.robotics.auton.automap;

import edu.nr.robotics.FieldData;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.AutoChoosers;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.StartAutoCommand;
import edu.nr.robotics.auton.autoroutes.StartPosMiddleToLeftSwitchProfilingCommand;
import edu.nr.robotics.auton.autoroutes.SwitchLeftToBlockCommand;
import edu.nr.robotics.auton.autoroutes.SwitchLeftToScaleLeftProfilingCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class StarPosMiddleSwitchLeftCommand extends CommandGroup {
	
	public StarPosMiddleSwitchLeftCommand() {
		
		//Checks
		addSequential(new ConditionalCommand(new StartPosMiddleToLeftSwitchProfilingCommand()) {
			
			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left;
			}
		});
		
		addSequential(new ConditionalCommand(new SwitchLeftToBlockCommand()) {

			@Override
			protected boolean condition() {
				return FieldData.getInstance().nearSwitch == Direction.left && StartAutoCommand.selectedScale == Scale.yes;
			}
			
		});
		
		
	}

}
