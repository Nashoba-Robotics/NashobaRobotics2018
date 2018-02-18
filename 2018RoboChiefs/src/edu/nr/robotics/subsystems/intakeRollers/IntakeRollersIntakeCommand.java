package edu.nr.robotics.subsystems.intakeRollers;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class IntakeRollersIntakeCommand extends NRCommand {
	
	public IntakeRollersIntakeCommand() {
		super(IntakeRollers.getInstance());
	}
	
	public void onStart() {
		IntakeRollers.getInstance().setMotorSpeedPercent(IntakeRollers.VEL_PERCENT_HIGH_INTAKE_ROLLERS, IntakeRollers.VEL_PERCENT_LOW_INTAKE_ROLLERS);
	}
	
	public void onEnd() {
		IntakeRollers.getInstance().disable();
	}
	
	public boolean isFinishedNR() {
		return (!EnabledSensors.intakeSensorLeft.get() && !EnabledSensors.intakeSensorRight.get());
	}

}
