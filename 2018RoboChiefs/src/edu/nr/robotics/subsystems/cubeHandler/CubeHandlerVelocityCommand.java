package edu.nr.robotics.subsystems.cubeHandler;

import edu.nr.lib.commandbased.NRCommand;

public class CubeHandlerVelocityCommand extends NRCommand {
	
private double percent;
	
	public CubeHandlerVelocityCommand(double percent) {
		super(CubeHandler.getInstance());
		this.percent = percent;
	}

	@Override
	protected void onStart() {
		CubeHandler.getInstance().setMotorSpeedPercent(percent);
	}

	@Override
	protected boolean isFinishedNR() {
		return true;
	}

}
