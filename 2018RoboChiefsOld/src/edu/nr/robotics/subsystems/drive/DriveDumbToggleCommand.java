package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.EnabledSubsystems;

public class DriveDumbToggleCommand extends NRCommand {

	public DriveDumbToggleCommand() {
		
	}
	
	protected void onStart() {
		EnabledSubsystems.DRIVE_DUMB_ENABLED = !EnabledSubsystems.DRIVE_DUMB_ENABLED;
	}
	
	protected boolean isFinishedNR() {
		return true;
	}
}
