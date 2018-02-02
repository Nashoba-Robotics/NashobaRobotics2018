package edu.nr.robotics.subsystems.sensors;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.network.LimelightNetworkTable.Pipeline;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class EnableLimelightCommand extends NRCommand {

	private boolean bool;
	
	public EnableLimelightCommand(boolean bool) {
		this.bool = bool;
	}
	
	@Override
	protected void onStart() {
		EnabledSensors.limelightEnabled = bool;
		LimelightNetworkTable.getInstance().setPipeline(Pipeline.PowerCube);
		//LimelightNetworkTable.getInstance().lightLED(bool);
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
