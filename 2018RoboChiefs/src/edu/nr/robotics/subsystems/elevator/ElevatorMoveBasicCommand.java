package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorMoveBasicCommand extends NRCommand {

	private Distance height;
	private double percent;
	private Distance initialPos;

	public ElevatorMoveBasicCommand(Distance height, double percent) {
		super(Elevator.getInstance());
		this.height = height;
		this.percent = percent;
	}

	@Override
	protected void onStart() {
		initialPos = Elevator.getInstance().getPosition();
		Elevator.getInstance().setMotorSpeedPercent(Math.abs(percent) * height.signum());
	}

	@Override
	protected void onEnd() {
		
	}
	
	@Override
	protected boolean isFinishedNR() {
		return (Elevator.getInstance().getPosition().sub(initialPos)).abs()
				.lessThan(Elevator.PROFILE_DELTA_POS_THRESHOLD_ELEVATOR);
	}
}
