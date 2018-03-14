package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.robotics.subsystems.climber.Climber;
import edu.nr.robotics.subsystems.elevator.Elevator;

public class ClimbButtonCommand extends NRCommand {
	
	public ClimbButtonCommand() {
		super(new NRSubsystem[] {Climber.getInstance(), Elevator.getInstance()});
	}
	
	@Override
	protected void onStart() {
		Elevator.getInstance().setMotorPercentRaw(0);
		Climber.getInstance().setMotorSpeedPercent(Climber.CLIMB_PERCENT);
	}
	
	@Override
	protected void onEnd() {
		Climber.getInstance().setMotorSpeedPercent(0);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}
	
}