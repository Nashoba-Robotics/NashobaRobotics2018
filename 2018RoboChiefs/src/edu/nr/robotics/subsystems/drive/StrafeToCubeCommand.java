package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;

public class StrafeToCubeCommand extends NRCommand {

	private Direction direction;
	GyroCorrection gyro;
	
	public StrafeToCubeCommand(Direction direction) {
		super(Drive.getInstance());
		this.direction = direction;
		gyro = new GyroCorrection();
	}
	
	@Override
	protected void onStart() {
		gyro.reset();
		new EnableLimelightCommand(true).start();
	}
	
	@Override
	protected void onExecute() {
		double turnValue = gyro.getTurnValue(Drive.kP_thetaOneD, false);
		if(direction == Direction.left) {
			Drive.getInstance().setMotorSpeedInPercent(-turnValue, turnValue, -Drive.SENSOR_STRAFE_PERCENT);
		} else {
			Drive.getInstance().setMotorSpeedInPercent(-turnValue, turnValue, Drive.SENSOR_STRAFE_PERCENT);
		}
	}
	
	@Override
	protected void onEnd() {
		Drive.getInstance().disable();
		new EnableLimelightCommand(false).start();
	}

	@Override
	protected boolean isFinishedNR() {
		return LimelightNetworkTable.getInstance().getHorizOffset().lessThan(Drive.DRIVE_ANGLE_THRESHOLD) && 
				!LimelightNetworkTable.getInstance().getHorizOffset().equals(Angle.ZERO);
	}
}
