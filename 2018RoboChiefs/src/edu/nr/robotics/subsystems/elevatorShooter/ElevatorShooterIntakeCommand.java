package edu.nr.robotics.subsystems.elevatorShooter;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorShooterIntakeCommand extends NRCommand {
	
	private double initIntakeTime = 0;
	private boolean isIntaking = false;
	private boolean intakeFinished = false;
	
	public ElevatorShooterIntakeCommand() {
		super(ElevatorShooter.getInstance());
	}
	
	@Override
	protected void onStart() {
		ElevatorShooter.getInstance().setMotorSpeedPercent(-ElevatorShooter.VEL_PERCENT_TRANSFER);
		initIntakeTime = 0;
		isIntaking = false;
		intakeFinished = false;
	}
	
	@Override
	protected void onExecute() {
		if (ElevatorShooter.getInstance().getCurrent() >= ElevatorShooter.SENSOR_BROKEN_CURRENT) {
			if (!isIntaking) {
				isIntaking = true;
				initIntakeTime = Timer.getFPGATimestamp();	
			
			} else {
				if ((Timer.getFPGATimestamp() - initIntakeTime) * 1000 >= ElevatorShooter.PEAK_CURRENT_DURATION_ELEVATOR_SHOOTER) {
					intakeFinished = true;
				}
			}
			
		} else {
			isIntaking = false;
		}
	}
	
	@Override
	protected void onEnd() {
		ElevatorShooter.getInstance().disable();
	}
	@Override
	protected boolean isFinishedNR() {
		return !EnabledSensors.elevatorSensor.get() || intakeFinished;
	}
}
