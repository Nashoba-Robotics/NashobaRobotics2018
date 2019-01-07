package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;

public class ElevatorPercentRawCommand extends NRCommand {

	private double percent;
	
	public ElevatorPercentRawCommand(double percent) {
		super(Elevator.getInstance());
		this.percent = percent;
	}
	
	@Override
	protected void onStart() {
		Elevator.getInstance().setMotorPercentRaw(percent);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return Elevator.getInstance().getPosition().lessThan(Elevator.SWITCH_HEIGHT_ELEVATOR.mul(0.5));
	}
}
