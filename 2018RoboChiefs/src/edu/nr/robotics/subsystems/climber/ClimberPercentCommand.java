package edu.nr.robotics.subsystems.climber;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomDropCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorPercentRawCommand;

public class ClimberPercentCommand extends NRCommand {
	
	public ClimberPercentCommand() {
		super(Climber.getInstance());
		
	}
	
	@Override
	protected void onStart() {
		Climber.getInstance().setMotorSpeedPercent(Climber.CLIMB_PERCENT);
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
