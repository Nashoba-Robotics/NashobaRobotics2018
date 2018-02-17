package edu.nr.robotics.subsystems.intakeRollers;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class IntakeRollersVelocitySmartDashboardCommand extends NRCommand {

	public IntakeRollersVelocitySmartDashboardCommand() {
		super(IntakeRollers.getInstance());
	}

	@Override
	protected void onStart() {
		IntakeRollers.getInstance().setMotorSpeedPercent(IntakeRollers.VEL_PERCENT_HIGH_INTAKE_ROLLERS, IntakeRollers.VEL_PERCENT_LOW_INTAKE_ROLLERS);
	}

	@Override
	protected void onEnd() {
		IntakeRollers.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return !EnabledSensors.intakeSensorLeft.get() && !EnabledSensors.intakeSensorRight.get();
	}
}