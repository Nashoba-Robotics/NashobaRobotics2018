package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.robotics.OI;
import edu.nr.robotics.Robot;

public class DriveJoystickCommand extends JoystickCommand {

	double prevTime = 0;
	
	private GyroCorrection gyroCorrection;

	public DriveJoystickCommand() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		gyroCorrection = new GyroCorrection();
	}

	@Override
	public void onExecute() {
		
		double dt = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - prevTime;
		prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
				
		switch (OI.driveMode) {
		case arcadeDrive:
			double moveValue = OI.getInstance().getArcadeMoveValue();
			double rotateValue = OI.getInstance().getArcadeTurnValue();
						
			moveValue = NRMath.powWithSign(moveValue, 3);
			rotateValue = NRMath.powWithSign(rotateValue, 3);
			
			if (Math.abs(rotateValue) < 0.05 && (Math.abs(moveValue) > 0.05)) {
				rotateValue = -gyroCorrection.getTurnValue(Drive.kP_thetaOneD, false);
			} else {
				gyroCorrection.clearInitialValue();
			}
			
			/*if (EnabledSensors.floorSensorEnabled && EnabledSensors.floorCounter.get() > 0) {
				Drive.getInstance().setMotorSpeedInPercent(0, 0, 0);
			} else {
				Drive.getInstance().arcadeDrive(moveValue * OI.getInstance().getDriveSpeedMultiplier(), rotateValue * OI.getInstance().getDriveSpeedMultiplier(), hValue * OI.getInstance().getDriveSpeedMultiplier());
			}*/
			Drive.getInstance().arcadeDrive(moveValue * OI.getInstance().getDriveSpeedMultiplier(), rotateValue * OI.getInstance().getDriveSpeedMultiplier());

									
			break;
		
		case tankDrive:
			double left = OI.getInstance().getTankLeftValue();
			double right = OI.getInstance().getTankRightValue();
			
			left = (Math.abs(left) + Math.abs(right)) / 2 * Math.signum(left);
			right = (Math.abs(left) + Math.abs(right)) / 2 * Math.signum(right);
			
			right = NRMath.powWithSign(right, 3);
			left = NRMath.powWithSign(left, 3);
			
			/*if (EnabledSensors.floorSensorEnabled && EnabledSensors.floorCounter.get() > 0) {
				Drive.getInstance().setMotorSpeedInPercent(0, 0, 0);
			} else {
				Drive.getInstance().tankDrive(OI.getInstance().getDriveSpeedMultiplier() * left, -OI.getInstance().getDriveSpeedMultiplier() * right, OI.getInstance().getDriveSpeedMultiplier() * hDrive);
			}*/
			break;
			
		case cheesyDrive:
			double cheesyMoveValue = OI.getInstance().getArcadeMoveValue();
			double cheesyRotateValue = OI.getInstance().getArcadeTurnValue();
			
			cheesyMoveValue = NRMath.powWithSign(cheesyMoveValue, 3);
			cheesyRotateValue = NRMath.powWithSign(cheesyRotateValue, 3);
			
			/*if (Math.abs(cheesyRotateValue) < 0.05 && (Math.abs(cheesyMoveValue) > 0.05 || Math.abs(cheesyHValue) > 0.05)) {
				cheesyRotateValue = gyroCorrection.getTurnValue(Drive.kP_thetaOneD, false);
			} else {
				gyroCorrection.clearInitialValue();
			}*/
			
			/*if (EnabledSensors.floorSensorEnabled && EnabledSensors.floorCounter.get() > 0) {
				Drive.getInstance().setMotorSpeedInPercent(0, 0, 0);
			} else {
				Drive.getInstance().cheesyDrive(cheesyMoveValue, cheesyRotateValue, cheesyHValue);
			}*/
			
			Drive.getInstance().cheesyDrive(cheesyMoveValue, cheesyRotateValue);
			
			break;
		}
		
	}

	@Override
	public boolean shouldSwitchToJoystick() {
		if (!Robot.getInstance().isAutonomous()) {
		
			if((OI.driveMode == Drive.DriveMode.arcadeDrive) || (OI.driveMode == Drive.DriveMode.cheesyDrive)) {
				return OI.getInstance().isArcadeNonZero();
			} else {
				return OI.getInstance().getTankLeftValue() != 0 || OI.getInstance().getTankRightValue() != 0;
			}
		}
		return false;
	}

	@Override
	public long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}
}
