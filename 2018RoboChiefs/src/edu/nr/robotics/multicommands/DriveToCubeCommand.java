package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.nr.robotics.subsystems.sensors.EnableIntakeSensorCommand;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;;

public class DriveToCubeCommand extends NRCommand {
	
	public Angle STOP_LIMELIGHT_TRACKING_ANGLE = Angle.ZERO; //TODO; Find STOP_LIMELIGHT_TRACKING_ANGLE
	
	private boolean stoppedTracking = false;
	
	private GyroCorrection gyro;
	
	public DriveToCubeCommand() {
		super(new NRSubsystem[] {Drive.getInstance(), IntakeElevator.getInstance()});
		gyro = new GyroCorrection();
	}
	
	@Override
	protected void onStart() {
		new EnableLimelightCommand(true).start();
		new EnableIntakeSensorCommand(true).start();
		Drive.getInstance().disable();
		IntakeRollers.getInstance().setMotorSpeedPercent(IntakeRollers.VEL_PERCENT_INTAKE_ROLLERS);
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
			headingAdjustment = 0;
		}
		else {
			headingAdjustment = LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.DEGREE) 
					* Drive.kP_thetaOneD;
		}
		
		double outputLeft, outputRight;
				
		outputLeft = -headingAdjustment;
		outputRight = headingAdjustment;
		
		if (LimelightNetworkTable.getInstance().getHorizOffset().abs().lessThan(Drive.DRIVE_ANGLE_THRESHOLD)) {
			outputLeft += Drive.DRIVE_TO_CUBE_PERCENT;
			outputRight += Drive.DRIVE_TO_CUBE_PERCENT;
		}
		
		Drive.getInstance().pidWrite(outputLeft, outputRight, 0);
	}

	@Override
	protected void onEnd() {
		new EnableLimelightCommand(false);
		new EnableIntakeSensorCommand(false).start();
		Drive.getInstance().disable();
		IntakeRollers.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		//TODO: DriveToCubeCommand get intake sensor to finish
		return false;
	}
}
