package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.interfaces.TriplePIDOutput;
import edu.nr.lib.units.Angle;

public class TurnPIDSmartDashboardCommand extends NRCommand {
		
	private TriplePIDOutput out;
	private Angle initialAngle;
	private GyroCorrection gyro;
	
	public TurnPIDSmartDashboardCommand() {
		super(Drive.getInstance());
		this.out = Drive.getInstance();
	}
	
	@Override
	public void onStart() {
		gyro = new GyroCorrection(Drive.angleToTurn, Drive.drivePercent, Drive.getInstance());
		out.pidWrite(0, 0, 0);
		initialAngle = gyro.getAngleError().sub(Drive.angleToTurn);
	}
	
	@Override
	public void onExecute() {
		
		double headingAdjustment = gyro.getTurnValue(Drive.kP_thetaOneD, true);
		if (Math.abs(headingAdjustment) < Drive.MIN_PROFILE_TURN_PERCENT) {
			headingAdjustment = Drive.MIN_PROFILE_TURN_PERCENT * Math.signum(headingAdjustment);
		}
		
		double outputLeft, outputRight;
		
		outputLeft = -headingAdjustment;
		outputRight = headingAdjustment;
		
		out.pidWrite(outputLeft, outputRight, 0);
		
	}
	
	@Override
	public boolean isFinishedNR() {
		
		boolean finished;
		if (Drive.exact == true) {
			finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftPosition())).abs()
					.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightPosition())).abs()
					.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (initialAngle.sub(gyro.getAngleError())).abs().lessThan(Drive.DRIVE_ANGLE_THRESHOLD);
		} else {
			finished = (initialAngle.sub(gyro.getAngleError())).abs().lessThan(Drive.DRIVE_ANGLE_THRESHOLD);	
		}
		return finished;
	}

}
