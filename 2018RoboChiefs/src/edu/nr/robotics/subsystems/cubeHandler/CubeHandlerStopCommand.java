package edu.nr.robotics.subsystems.cubeHandler;

import edu.nr.lib.commandbased.NRCommand;

public class CubeHandlerStopCommand extends NRCommand {


	/**
	 * Sets motor speed to 0
	 */
	public CubeHandlerStopCommand() {
		super(CubeHandler.getInstance());
	}
	
	@Override
	protected void onStart() {
		CubeHandler.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
	
}
