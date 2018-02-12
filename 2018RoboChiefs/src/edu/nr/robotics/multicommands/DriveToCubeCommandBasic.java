package edu.nr.robotics.multicommands;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class DriveToCubeCommandBasic extends NRCommand {

	private boolean hasStartedForward = false;
	
	private boolean finished = false;
	
	private GyroCorrection gyro;
	
	public DriveToCubeCommandBasic() {
		super(new NRSubsystem[] {Drive.getInstance(), IntakeElevator.getInstance()});
		gyro = new GyroCorrection();
	}
	
	@Override
	protected void onStart() {
		hasStartedForward = false;
		new EnableLimelightCommand(true).start();
		IntakeRollers.getInstance().setMotorSpeedPercent(IntakeRollers.VEL_PERCENT_HIGH_INTAKE_ROLLERS, IntakeRollers.VEL_PERCENT_LOW_INTAKE_ROLLERS);
		Drive.getInstance().setMotorSpeedInPercent(0, 0, 0);
		gyro.reset();
		
		if ((IntakeElevator.getInstance().getPosition().sub(IntakeElevator.INTAKE_HEIGHT)).abs().greaterThan(IntakeElevator.PROFILE_DELTA_POS_THRESHOLD_INTAKE_ELEVATOR)) {
			finished = true;
		} else {
			finished = false;
		}
	}
	
	@Override
	protected void onExecute() {
		
		double headingAdjustment = 0;
		double outputLeft = 0;
		double outputRight = 0;
		
		if (!hasStartedForward) {
			
			headingAdjustment = ((-Math.cos(LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.RADIAN) / ((Drive.DRIVE_STOP_ANGLE.get(Angle.Unit.DEGREE) / 90) * 3)) * (1 - Drive.MIN_PROFILE_TURN_PERCENT)) + 1 + Drive.MIN_PROFILE_TURN_PERCENT) * -LimelightNetworkTable.getInstance().getHorizOffset().signum();
			
			if (Math.abs(headingAdjustment) < Drive.MIN_PROFILE_TURN_PERCENT) {
				headingAdjustment = Drive.MIN_PROFILE_TURN_PERCENT * Math.signum(headingAdjustment);
			
			}
			
			outputLeft = -headingAdjustment;
			outputRight = headingAdjustment;
			
		}
		
		
		if (LimelightNetworkTable.getInstance().getHorizOffset().abs().lessThan(Drive.DRIVE_ANGLE_THRESHOLD)
				&& !LimelightNetworkTable.getInstance().getHorizOffset().abs().equals(Angle.ZERO) 
				&& !hasStartedForward) {
			hasStartedForward = true;
			gyro.reset();
		}
		
		if (hasStartedForward = true) {
			outputLeft = -headingAdjustment;
			outputRight = headingAdjustment;
			
			outputLeft += Drive.DRIVE_TO_CUBE_PERCENT;
			outputRight += Drive.DRIVE_TO_CUBE_PERCENT;
		}
		
		Drive.getInstance().pidWrite(outputLeft, outputRight, 0);
	}

	@Override
	protected void onEnd() {
		new EnableLimelightCommand(false);
		Drive.getInstance().setMotorSpeedInPercent(0, 0, 0);
		IntakeRollers.getInstance().setMotorSpeedPercent(0, 0);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return !EnabledSensors.intakeSensor.get() || finished;
	}
	
}
