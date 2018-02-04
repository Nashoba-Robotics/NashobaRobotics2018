package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.subsystems.sensors.EnablePortalSensorsCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class StrafeToPortalCommand extends NRCommand {

	Direction direction;
	private GyroCorrection gyro;

	
	public StrafeToPortalCommand(Direction direction) {
		super(Drive.getInstance());
		this.direction = direction;
		gyro = new GyroCorrection();
	}
	
	@Override
	protected void onStart() {
		new EnablePortalSensorsCommand(true).start();
		gyro.reset();
	}
	
	@Override 
	protected void onExecute() {
		double turnValue = gyro.getTurnValue(Drive.kP_thetaOneD, false);
		if (direction == Direction.left) {	
			Drive.getInstance().setMotorSpeedInPercent(-turnValue, turnValue, -Drive.SENSOR_STRAFE_PERCENT);
		} else {
			Drive.getInstance().setMotorSpeedInPercent(-turnValue, turnValue, Drive.SENSOR_STRAFE_PERCENT);
		}
	}
	
	@Override
	protected void onEnd() {
		new EnablePortalSensorsCommand(false).start();
		Drive.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return EnabledSensors.portalReached;
	}

}
