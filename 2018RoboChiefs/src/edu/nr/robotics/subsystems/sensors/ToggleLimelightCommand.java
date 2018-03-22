package edu.nr.robotics.subsystems.sensors;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.network.LimelightNetworkTable;

public class ToggleLimelightCommand extends NRCommand {

	public ToggleLimelightCommand() {
		
	}
	
	@Override
	protected void onStart() {
		if (LimelightNetworkTable.getInstance().getLED() == 0) {
			LimelightNetworkTable.getInstance().lightLED(true);
		} else {
			LimelightNetworkTable.getInstance().lightLED(false);
		}
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
