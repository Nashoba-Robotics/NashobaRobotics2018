package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorBottomDropCommand extends NRCommand {

	public ElevatorBottomDropCommand() {
		super(Elevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		Elevator.getInstance().setMotorPercentRaw(Elevator.DROP_PERCENT_ELEVATOR);
	}
	
	@Override
	protected void onEnd() {
		Elevator.getInstance().setMotorPercentRaw(0);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return Elevator.getInstance().getPosition().lessThan(new Distance(4, Distance.Unit.INCH));
	}
}
