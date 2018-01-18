package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.Gyro;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.TriplePIDOutput;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;

public class TurnPIDCommand extends NRCommand {

	private static final Angle ANGLE_THRESHOLD = new Angle(0.1, Angle.Unit.DEGREE);
	
	private TriplePIDOutput out;
	private Angle angleToTurn;
	private Angle initialAngle;
	private GyroCorrection gyro;
	private double kP_theta;
	private double turnPercent;
	
	public TurnPIDCommand(TriplePIDOutput out, Angle angleToTurn, double turnPercent, double kP_theta) {
		this.out = out;
		this.angleToTurn = angleToTurn;
		this.turnPercent = turnPercent;
		this.kP_theta = kP_theta;
	}
	
	@Override
	public void onStart() {
		gyro = new GyroCorrection(angleToTurn, turnPercent, Drive.getInstance());
		out.pidWrite(0, 0, 0);
		initialAngle = gyro.getAngleError().sub(angleToTurn);
	}
	
	@Override
	public void onExecute() {
		
		double headingAdjustment = gyro.getTurnValue(kP_theta);	
		
		double outputLeft, outputRight;
		
		outputLeft = -headingAdjustment;
		outputRight = headingAdjustment;
		
		out.pidWrite(outputLeft, outputRight, 0);
		
	}
	
	@Override
	public boolean isFinishedNR() {
		
		boolean finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftPosition())).abs()
		.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
		&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightPosition())).abs()
		.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
		&& (initialAngle.sub(gyro.getAngleError())).abs().lessThan(ANGLE_THRESHOLD);
		return finished;
	}

}
