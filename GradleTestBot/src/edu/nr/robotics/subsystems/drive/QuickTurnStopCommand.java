package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;

public class QuickTurnStopCommand extends NRCommand {

	public QuickTurnStopCommand() {
		
	}
	
	@Override
	public void onStart() {
		Drive.isQuickTurn = true;
	}
}
