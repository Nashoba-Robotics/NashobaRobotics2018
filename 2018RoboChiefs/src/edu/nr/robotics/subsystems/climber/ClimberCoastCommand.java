package edu.nr.robotics.subsystems.climber;

import edu.nr.lib.commandbased.NRCommand;

public class ClimberCoastCommand extends NRCommand {

	private boolean bool;
	
	public ClimberCoastCommand(boolean bool) {
		super(Climber.getInstance());
		this.bool = bool;
	}
	
	@Override
	protected void onStart() {
		Climber.getInstance().setCoastMode(bool);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
