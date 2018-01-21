package edu.nr.robotics.subsystems.climber;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ClimberHoldPositionCommand extends NRCommand {

	private Distance initialPos;
	
	public ClimberHoldPositionCommand() {
		super(Climber.getInstance());
	}
	
	@Override
	protected void onStart() {
		initialPos = Climber.getInstance().getPosition();
	}
	
	@Override
	protected void onExecute() {
		Climber.getInstance().setPosition(initialPos);
	}
	
	protected boolean isFinishedNR() {
		return false;
	}
}
