package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.Gyro;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;

public class TurnPIDCommand extends NRCommand {

	private static final Angle ANGLE_THRESHOLD = new Angle(0.01, Angle.Unit.DEGREE);
	
	private DoublePIDOutput out;
	Angle angleToTurn;
	Angle initialAngle;
	GyroCorrection gyro;
	
	public TurnPIDCommand(DoublePIDOutput out, Angle angleToTurn) {
		this.out = out;
		this.angleToTurn = angleToTurn;
		gyro = new GyroCorrection(angleToTurn);
	}
	
	@Override
	public void onStart() {
		out.pidWrite(0, 0);
		initialAngle = gyro.getAngleError().sub(angleToTurn);
	}
	
	@Override
	public void onExecute() {
		double headingAdjustment =  gyro.getTurnValue(Drive.kP_thetaOneD); 
		
		double outputLeft, outputRight;
		
		outputLeft = -headingAdjustment;
		outputRight = headingAdjustment;
		
		out.pidWrite(outputLeft, outputRight);
	}
	
	@Override
	public boolean isFinishedNR() {
		
		/*boolean finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftDistance())).abs()
		.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
		&& (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getLeftDistance())).abs()
		.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
		&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightDistance())).abs()
		.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
		&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getRightDistance())).abs()
		.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
		&& (initialAngle.sub(gyro.getAngleError())).abs().lessThan(ANGLE_THRESHOLD);
		return finished;*/
		return false;
	}

}
