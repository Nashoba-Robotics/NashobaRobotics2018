package edu.nr.robotics.subsystems.climber;

import edu.nr.lib.commandbased.NRCommand;

public class ClimberCoastCommand extends NRCommand {

	boolean bool;
	
	public ClimberCoastCommand(boolean bool) {
		super(Climber.getInstance());
		this.bool = bool;
	}
	
	protected void onStart() {
		Climber.getInstance().setCoastMode(bool);
	}
	
	protected boolean isFinishedNR() {
		return true;
	}
}
