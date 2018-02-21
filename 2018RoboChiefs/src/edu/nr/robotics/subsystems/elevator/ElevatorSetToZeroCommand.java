package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;

public class ElevatorSetToZeroCommand extends NRCommand {
	
	public ElevatorSetToZeroCommand() {
		super(Elevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		Elevator.getInstance().setMotorSpeedPercent(0);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}
	
}
