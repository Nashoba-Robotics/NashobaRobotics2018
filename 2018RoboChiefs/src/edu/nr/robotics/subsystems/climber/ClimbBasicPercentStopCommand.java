package edu.nr.robotics.subsystems.climber;

import edu.nr.lib.commandbased.NRCommand;

public class ClimbBasicPercentStopCommand extends NRCommand {
	
	public ClimbBasicPercentStopCommand() {
		super(Climber.getInstance());
	}
	
	@Override
	protected void onStart() {
		Climber.getInstance().setMotorSpeedPercent(0);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}

}
