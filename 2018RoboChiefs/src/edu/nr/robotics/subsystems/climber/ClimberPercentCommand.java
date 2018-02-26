package edu.nr.robotics.subsystems.climber;

import edu.nr.lib.commandbased.NRCommand;

public class ClimberPercentCommand extends NRCommand {
	
	public ClimberPercentCommand() {
		super(Climber.getInstance());
		
	}
	
	@Override
	protected void onStart() {
		Climber.getInstance().setCoastMode(false);
		Climber.getInstance().setMotorSpeedPercent(Climber.CLIMB_PERCENT);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}

}
