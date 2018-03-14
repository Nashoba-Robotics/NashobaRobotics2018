package edu.nr.robotics.subsystems.climber;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomDropCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorPercentRawCommand;

public class ClimberPercentCommand extends NRCommand {
	
	private double percent;
	
	public ClimberPercentCommand(double percent) {
		super(Climber.getInstance());
		this.percent = percent;
	}
	
	@Override
	protected void onStart() {
		Climber.getInstance().setMotorSpeedPercent(percent);
	}
	
	@Override
	protected void onEnd() {
		new ElevatorPercentRawCommand(0).start();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}

}
