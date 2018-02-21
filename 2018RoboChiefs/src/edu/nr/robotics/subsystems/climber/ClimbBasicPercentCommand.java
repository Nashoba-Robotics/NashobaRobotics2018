package edu.nr.robotics.subsystems.climber;

import edu.nr.lib.commandbased.NRCommand;

public class ClimbBasicPercentCommand extends NRCommand {
	
	public ClimbBasicPercentCommand() {
		super(Climber.getInstance());
		
	}
	
	@Override
	protected void onStart() {
		Climber.getInstance().setMotorSpeedPercent(Climber.CLIMB_PERCENT);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}

}
