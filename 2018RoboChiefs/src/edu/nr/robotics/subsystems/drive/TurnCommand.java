package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.Gyro;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.TriplePIDOutput;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;

public class TurnCommand extends NRCommand {
	
	private TriplePIDOutput out;
	private Angle angleToTurn;
	private Angle initialAngle;
	private GyroCorrection gyro;
	private double turnPercent;
	
	public TurnCommand(TriplePIDOutput out, Angle angleToTurn, double turnPercent) {
		super(Drive.getInstance());
		this.out = out;
		this.angleToTurn = angleToTurn;
		this.turnPercent = turnPercent;
	}
	
	@Override
	public void onStart() {
		gyro = new GyroCorrection(angleToTurn, turnPercent);
		out.pidWrite(0, 0, 0);
		initialAngle = gyro.getAngleError().sub(angleToTurn);
	}
	
	@Override
	public void onExecute() {
		
		double headingAdjustment = gyro.getTurnValue(true);
		if (Math.abs(headingAdjustment) < Drive.MIN_PROFILE_TURN_PERCENT) {
			headingAdjustment = Drive.MIN_PROFILE_TURN_PERCENT * Math.signum(headingAdjustment);
		}
		
		double outputLeft, outputRight;
		
		outputLeft = -headingAdjustment;
		outputRight = headingAdjustment;
		
		out.pidWrite(outputLeft, outputRight, 0);
		
	}
	
	@Override
	public void onEnd() {
		Drive.getInstance().disable();
	}
	
	@Override
	public boolean isFinishedNR() {
		
		boolean finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftPosition())).abs()
					.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightPosition())).abs()
					.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (initialAngle.sub(gyro.getAngleError())).abs().lessThan(Drive.DRIVE_ANGLE_THRESHOLD);
		return finished;
	}

}
