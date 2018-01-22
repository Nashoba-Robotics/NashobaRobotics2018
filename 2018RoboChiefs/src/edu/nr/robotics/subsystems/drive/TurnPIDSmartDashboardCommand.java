package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.interfaces.TriplePIDOutput;
import edu.nr.lib.units.Angle;

public class TurnPIDSmartDashboardCommand extends NRCommand {
		
	private TriplePIDOutput out;
	private Angle initialAngle;
	private GyroCorrection gyro;
	private double kP_theta;
	
	public TurnPIDSmartDashboardCommand(TriplePIDOutput out, double kP_theta) {
		super(Drive.getInstance());
		this.out = out;
		this.kP_theta = kP_theta;
	}
	
	@Override
	public void onStart() {
		gyro = new GyroCorrection(Drive.angleToTurn, Drive.drivePercent, Drive.getInstance());
		out.pidWrite(0, 0, 0);
		initialAngle = gyro.getAngleError().sub(Drive.angleToTurn);
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
		
		boolean finished;
		if (Drive.exact == true) {
			finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftPosition())).abs()
					.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightPosition())).abs()
					.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (initialAngle.sub(gyro.getAngleError())).abs().lessThan(Drive.DRIVE_ANGLE_THRESHOLD);
		} else {
			finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftPosition())).abs()
					.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightPosition())).abs()
					.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
					&& (initialAngle.sub(gyro.getAngleError())).abs().lessThan(Drive.DRIVE_ANGLE_THRESHOLD);	
		}
		return finished;
	}

}
