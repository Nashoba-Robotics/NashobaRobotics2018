package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.interfaces.TriplePIDOutput;
import edu.nr.lib.motionprofiling.HDriveDiagonalProfiler;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;

public class TurnSmartDashboardCommand extends NRCommand {
		
	private TriplePIDOutput out;
	private Angle initialAngle;
	private GyroCorrection gyro;

	public TurnSmartDashboardCommand() {
		super(Drive.getInstance());
		this.out = Drive.getInstance();
	}
	
	@Override
	public void onStart() {
		gyro = new GyroCorrection(Drive.angleToTurn, Drive.drivePercent);
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
	public void onEnd() {
		Drive.getInstance().disable();
	}
	
	@Override
	public boolean isFinishedNR() {
		
		boolean finished;
		finished = Drive.getInstance().getLeftVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD)
					&& Drive.getInstance().getRightVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD)
					&& Drive.getInstance().getHVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD) && 
					(initialAngle.sub(gyro.getAngleError())).abs().lessThan(Drive.DRIVE_ANGLE_THRESHOLD);
		return finished;
	}

}
