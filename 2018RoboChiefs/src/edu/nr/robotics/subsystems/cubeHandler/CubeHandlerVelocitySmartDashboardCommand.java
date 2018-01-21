package edu.nr.robotics.subsystems.cubeHandler;

import edu.nr.lib.commandbased.NRCommand;

public class CubeHandlerVelocitySmartDashboardCommand extends NRCommand {
	
	public CubeHandlerVelocitySmartDashboardCommand() {
		super(CubeHandler.getInstance());
	}

	@Override
	protected void onStart() {
		CubeHandler.getInstance().setMotorSpeedPercent(CubeHandler.VEL_PERCENT_CUBE_HANDLER);
	}

	@Override
	protected boolean isFinishedNR() {
		return false;
	}

}