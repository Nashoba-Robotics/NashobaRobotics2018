package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;
import edu.wpi.first.wpilibj.Timer;

public class DriveToCubeCommandAdvanced extends NRCommand {
	
	public Angle STOP_LIMELIGHT_TRACKING_ANGLE = new Angle(-14, Angle.Unit.DEGREE);
	
	private boolean stoppedTracking = false;
	private boolean hasStartedForward = false;
		
	private GyroCorrection gyro;
	
	public DriveToCubeCommandAdvanced() {
		//super(new NRSubsystem[] {Drive.getInstance(), IntakeElevator.getInstance()});
		super(Drive.getInstance());
		gyro = new GyroCorrection();
	}
	
	@Override
	protected void onStart() {
		hasStartedForward = false;
		stoppedTracking = false;
		new EnableLimelightCommand(true).start();
		//IntakeRollers.getInstance().setMotorSpeedPercent(IntakeRollers.VEL_PERCENT_INTAKE_ROLLERS);
		gyro.reset();
	}
	
	@Override
	protected void onExecute() {
				
		double headingAdjustment;
		
		if (stoppedTracking) {
			headingAdjustment = gyro.getTurnValue(Drive.kP_thetaOneD, false);
		}
		else if (LimelightNetworkTable.getInstance().getVertOffsetAngle().lessThan(STOP_LIMELIGHT_TRACKING_ANGLE)) {
			stoppedTracking = true;
			gyro.reset();
			headingAdjustment = gyro.getTurnValue(Drive.kP_thetaOneD, false);
		}
		else {
				
			headingAdjustment = ((-Math.cos(LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.RADIAN)
					/ (Drive.DRIVE_STOP_ANGLE.get(Angle.Unit.DEGREE) / 90 * 3)) * (1 - Drive.MIN_PROFILE_TURN_PERCENT))
					+ 1 + Drive.MIN_PROFILE_TURN_PERCENT)
					* -LimelightNetworkTable.getInstance().getHorizOffset().signum();
			
			if (Math.abs(headingAdjustment) < Drive.MIN_PROFILE_TURN_PERCENT) {
				headingAdjustment = Drive.MIN_PROFILE_TURN_PERCENT * Math.signum(headingAdjustment);
			}
		}
		
		double outputLeft, outputRight;
				
		outputLeft = -headingAdjustment;
		outputRight = headingAdjustment;
		
		if (LimelightNetworkTable.getInstance().getHorizOffset().abs().lessThan(Drive.DRIVE_ANGLE_THRESHOLD) &&
				!LimelightNetworkTable.getInstance().getHorizOffset().abs().equals(Angle.ZERO) &&
				!hasStartedForward) {
			hasStartedForward = true;
		}
		
		if (hasStartedForward) {
			outputLeft += Drive.DRIVE_TO_CUBE_PERCENT;
			outputRight += Drive.DRIVE_TO_CUBE_PERCENT;
		}
		
		Drive.getInstance().pidWrite(outputLeft, outputRight, 0);	
	}

	@Override
	protected void onEnd() {
		Drive.getInstance().setMotorSpeedInPercent(0, 0, 0);
		new EnableLimelightCommand(false);
		//IntakeRollers.getInstance().setMotorSpeedPercent(0);
	}
	
	@Override
	protected boolean isFinishedNR() {
		//return !EnabledSensors.intakeSensor.get();
		return stoppedTracking;
	}
}
