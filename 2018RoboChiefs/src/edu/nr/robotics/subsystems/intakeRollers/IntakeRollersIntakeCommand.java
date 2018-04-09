package edu.nr.robotics.subsystems.intakeRollers;


import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.wpi.first.wpilibj.Timer;

public class IntakeRollersIntakeCommand extends NRCommand {
	
	private double initStuckTime = 0;
	private boolean stuck = false;
	private boolean stuckTooLong = false;
	
	private boolean hasStopped = false;
	private double initStopTime = 0;
	
	private boolean isSensorBroken = false;
	
	public static final Time TIME_STOPPED_AFTER_STUCK = new Time(0.5, Time.Unit.SECOND);
	
	public IntakeRollersIntakeCommand() {
		super(IntakeRollers.getInstance());
	}
	
	@Override
	public void onStart() {
		IntakeRollers.getInstance().setMotorSpeedPercent(IntakeRollers.VEL_PERCENT_HIGH_INTAKE_ROLLERS, IntakeRollers.VEL_PERCENT_LOW_INTAKE_ROLLERS);
		initStuckTime = 0;
		initStopTime = 0;
		stuck = false;
		stuckTooLong = false;
		hasStopped = false;
	}
	
	@Override
	protected void onExecute() {
		if (!stuckTooLong && IntakeRollers.getInstance().getCurrentLeft() > IntakeRollers.CURRENT_PEAK_INTAKE_ROLLERS || IntakeRollers.getInstance().getCurrentRight() > IntakeRollers.CURRENT_PEAK_INTAKE_ROLLERS) {
			if (!stuck) {
				stuck = true;
				initStuckTime = Timer.getFPGATimestamp();
			} else {
				// Multiplied by 1000 to go from seconds to milliseconds
				if((Timer.getFPGATimestamp() - initStuckTime) * 1000 > IntakeRollers.PEAK_CURRENT_DURATION_INTAKE_ROLLERS) {
					stuckTooLong = true;
				}
			}
			
		} else {
			stuck = false;
		}
		
		if (stuckTooLong) {
			if(IntakeRollers.getInstance().getCurrentLeft() > IntakeRollers.CURRENT_PEAK_INTAKE_ROLLERS && IntakeRollers.getInstance().getCurrentRight() > IntakeRollers.CURRENT_PEAK_INTAKE_ROLLERS) {
				isSensorBroken = true;
			}
			else if (!hasStopped) {
				hasStopped = true;
				IntakeRollers.getInstance().setMotorSpeedPercent(0, 0);
				initStopTime = Timer.getFPGATimestamp();
			} else {
				if (Timer.getFPGATimestamp() - initStopTime > TIME_STOPPED_AFTER_STUCK.get(Time.Unit.SECOND)) {
					initStuckTime = 0;
					initStopTime = 0;
					stuck = false;
					stuckTooLong = false;
					hasStopped = false;
					IntakeRollers.getInstance().setMotorSpeedPercent(IntakeRollers.VEL_PERCENT_HIGH_INTAKE_ROLLERS, IntakeRollers.VEL_PERCENT_LOW_INTAKE_ROLLERS);
				}
			}
		}
	}
	
	@Override
	public void onEnd() {
		IntakeRollers.getInstance().disable();
	}
	
	@Override
	public boolean isFinishedNR() {
		return (!EnabledSensors.intakeSensorLeft.get() && !EnabledSensors.intakeSensorRight.get()) || isSensorBroken /*|| IntakeRollers.getInstance().getCurrentLeft() > IntakeRollers.PEAK_CURRENT_INTAKE_ROLLERS || IntakeRollers.getInstance().getCurrentRight() > IntakeRollers.PEAK_CURRENT_INTAKE_ROLLERS*/;
	}

}
