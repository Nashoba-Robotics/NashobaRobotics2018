package edu.nr.robotics.subsystems.sensors;

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
		EnabledSensors.limelightEnabled = bool;
		LimelightNetworkTable.getInstance().setPipeline(Pipeline.PowerCube);

		if (bool) {
			LimelightNetworkTable.getInstance().lightLED(true);
			LimelightNetworkTable.getInstance().enable();
		} else {
			LimelightNetworkTable.getInstance().lightLED(false);
			LimelightNetworkTable.getInstance().disable();
		}
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
