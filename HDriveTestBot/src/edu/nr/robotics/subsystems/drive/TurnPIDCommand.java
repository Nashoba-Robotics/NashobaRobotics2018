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
	Angle angleToTurn;
	Angle initialAngle;
	GyroCorrection gyro;
	private double lastHeadingAdjustment;
	private double prevTime;
	
	public TurnPIDCommand(TriplePIDOutput out) {
		this.out = out;
	}
	
	@Override
	public void onStart() {
		this.angleToTurn = Drive.angleToTurn;
		gyro = new GyroCorrection(angleToTurn, 0.5, Drive.getInstance());
		out.pidWrite(0, 0, 0);
		initialAngle = gyro.getAngleError().sub(angleToTurn);
		lastHeadingAdjustment = 0;
		prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

	}
	
	@Override
	public void onExecute() {
		double dt = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - prevTime;
		prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		
		double headingAdjustment = gyro.getTurnValue(Drive.kP_thetaOneD);
		
		double errorDeriv = (headingAdjustment - lastHeadingAdjustment) / dt;		
		
		double outputLeft, outputRight;
		
		outputLeft = -headingAdjustment - errorDeriv * Drive.kDTurn;
		outputRight = headingAdjustment + errorDeriv * Drive.kDTurn;
		
		out.pidWrite(outputLeft, outputRight, 0);
		
		lastHeadingAdjustment = headingAdjustment;
	}
	
	@Override
	public boolean isFinishedNR() {
		
		boolean finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftDistance())).abs()
		.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
		//&& (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getLeftDistance())).abs()
		//.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
		&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightDistance())).abs()
		//.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
		//&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getRightDistance())).abs()
		.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
		&& (initialAngle.sub(gyro.getAngleError())).abs().lessThan(ANGLE_THRESHOLD);
		return finished;
	}

}
