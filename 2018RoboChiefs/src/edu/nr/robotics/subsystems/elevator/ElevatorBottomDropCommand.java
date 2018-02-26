package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;

public class ElevatorBottomDropCommand extends NRCommand {

	public ElevatorBottomDropCommand() {
		super(Elevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		Elevator.getInstance().setMotorPercentRaw(0);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return Elevator.getInstance().getPosition().lessThan(Elevator.SWITCH_HEIGHT_ELEVATOR.mul(0.5));
	}
}
