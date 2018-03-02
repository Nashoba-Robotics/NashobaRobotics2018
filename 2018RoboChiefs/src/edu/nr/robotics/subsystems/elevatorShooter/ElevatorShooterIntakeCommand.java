package edu.nr.robotics.subsystems.elevatorShooter;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class ElevatorShooterIntakeCommand extends NRCommand {
	
	public ElevatorShooterIntakeCommand() {
		super(ElevatorShooter.getInstance());
	}
	
	protected void onStart() {
		ElevatorShooter.getInstance().setMotorSpeedPercent(-ElevatorShooter.VEL_PERCENT_TRANSFER);
	}
	
	protected void onEnd() {
		ElevatorShooter.getInstance().disable();
	}
	
	protected boolean isFinishedNR() {
		return !EnabledSensors.elevatorSensor.get();
	}
}
