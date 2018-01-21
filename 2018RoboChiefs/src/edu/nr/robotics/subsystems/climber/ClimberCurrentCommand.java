package edu.nr.robotics.subsystems.climber;

import edu.nr.lib.commandbased.NRCommand;

public class ClimberCurrentCommand extends NRCommand {
	
	private double current;
	
	public ClimberCurrentCommand(double current) {
		super(Climber.getInstance());
		this.current = current;
	}
	
	@Override
	protected void onStart() {
		Climber.getInstance().setCurrent(current);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}

}
