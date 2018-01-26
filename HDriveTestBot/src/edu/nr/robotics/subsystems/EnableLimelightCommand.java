package edu.nr.robotics.subsystems;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.network.LimelightNetworkTable.Pipeline;

public class EnableLimelightCommand extends NRCommand {

	private boolean bool;
	
	public EnableLimelightCommand(boolean bool) {
		this.bool = bool;
	}
	
	@Override
	protected void onStart() {
		LimelightNetworkTable.getInstance().setPipeline(Pipeline.PowerCube);
		LimelightNetworkTable.getInstance().lightLED(bool);
		if (bool) {
			LimelightNetworkTable.getInstance().enable();
		} else {
			LimelightNetworkTable.getInstance().disable();
		}
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
