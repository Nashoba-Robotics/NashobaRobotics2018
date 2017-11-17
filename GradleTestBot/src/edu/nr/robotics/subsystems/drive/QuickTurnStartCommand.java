package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;

public class QuickTurnStartCommand extends NRCommand {
	
	public QuickTurnStartCommand() {
		
	}
	
	@Override
	public void onStart() {
		Drive.isQuickTurn = true;
	}
	
}
