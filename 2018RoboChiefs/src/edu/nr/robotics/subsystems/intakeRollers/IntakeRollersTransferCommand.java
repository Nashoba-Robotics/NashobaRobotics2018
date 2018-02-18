package edu.nr.robotics.subsystems.intakeRollers;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class IntakeRollersTransferCommand extends NRCommand {
	
	public IntakeRollersTransferCommand() {
		super(IntakeRollers.getInstance());
	}
	
	protected void onStart() {
		IntakeRollers.getInstance().setMotorSpeedPercent(IntakeRollers.VEL_PERCENT_HIGH_INTAKE_ROLLERS, IntakeRollers.VEL_PERCENT_HIGH_INTAKE_ROLLERS);
	}
	
	protected void onEnd() {
		IntakeRollers.getInstance().disable();
	}
	
	protected boolean isFinishedNR() {
		return (EnabledSensors.intakeSensorLeft.get() && EnabledSensors.intakeSensorRight.get());
	}

}
