package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;

public class QuickTurnCommand extends NRCommand {
	
	public QuickTurnCommand() {
		
	}
	
	@Override
	public void onStart() {
		Drive.isQuickTurn = true;
	}
	
	@Override
	public void onEnd() {
		Drive.isQuickTurn = false;
	}
	
}
