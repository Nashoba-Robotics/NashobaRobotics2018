package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.EnableLimelightCommand;

public class DriveToCubeCommand extends NRCommand {
	
	public Angle STOP_LIMELIGHT_TRACKING_ANGLE = new Angle(-13, Angle.Unit.DEGREE);
	public static final double DRIVE_TO_CUBE_PERCENT = 0.2;
	public static final Angle DRIVE_ANGLE_THRESHOLD = new Angle(1, Angle.Unit.DEGREE);
	
	private boolean stoppedTracking = false;
	private boolean hasStartedForward = false;
	
	private GyroCorrection gyro;
	
	public DriveToCubeCommand() {
		super(Drive.getInstance());
		gyro = new GyroCorrection();
	}
	
	@Override
	protected void onStart() {
		hasStartedForward = false;
		stoppedTracking = false;
		new EnableLimelightCommand(true).start();
		gyro.reset();
	}
	
	@Override
	protected void onExecute() {
		
		double headingAdjustment;
		
		if (stoppedTracking) {
			headingAdjustment = gyro.getTurnValue(Drive.kP_thetaOneD);
		}
		else if (LimelightNetworkTable.getInstance().getVertOffsetAngle().lessThan(STOP_LIMELIGHT_TRACKING_ANGLE)) {
			stoppedTracking = true;
			headingAdjustment = gyro.getTurnValue(Drive.kP_thetaOneD);
		}
		else {
			headingAdjustment = NRMath.powWithSign(-LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.DEGREE) 
					* Drive.kP_thetaOneD, 2);
			if (Math.abs(headingAdjustment) < 0.05) {
				headingAdjustment = 0.05 * Math.signum(headingAdjustment);
			}
		}
		
		double outputLeft, outputRight;
				
		outputLeft = -headingAdjustment;
		outputRight = headingAdjustment;
		
		if (LimelightNetworkTable.getInstance().getHorizOffset().abs().lessThan(DRIVE_ANGLE_THRESHOLD) &&
				!LimelightNetworkTable.getInstance().getHorizOffset().abs().equals(Angle.ZERO) &&
				!hasStartedForward) {
			hasStartedForward = true;
		}
		
		if (hasStartedForward) {
			outputLeft += DRIVE_TO_CUBE_PERCENT;
			outputRight += DRIVE_TO_CUBE_PERCENT;
		}
		
		Drive.getInstance().pidWrite(outputLeft, outputRight, 0);
	}

	@Override
	protected void onEnd() {
		new EnableLimelightCommand(false);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}
}
