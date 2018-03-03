package edu.nr.robotics.subsystems.climber;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomDropCommand;

public class ClimberStopCommand extends NRCommand {
	
	public ClimberStopCommand() {
		super(Climber.getInstance());
	}
	
	@Override
	protected void onStart() {
		Climber.getInstance().setMotorSpeedPercent(0);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}

}
