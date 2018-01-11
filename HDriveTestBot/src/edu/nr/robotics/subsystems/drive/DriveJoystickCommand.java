package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.OI;

public class DriveJoystickCommand extends JoystickCommand {

	GyroCorrection gyroCorrection;

	public DriveJoystickCommand() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		gyroCorrection = new GyroCorrection();
	}

	@Override
	public void onExecute() {
		switch (OI.getInstance().DriveMode.getSelected()) {
		case arcadeDrive:
			double moveValue = OI.getInstance().getArcadeMoveValue();
			double rotateValue = OI.getInstance().getArcadeTurnValue();
			double hValue = OI.getInstance().getArcadeHValue();
			
			moveValue = NRMath.powWithSign(moveValue, 3);
			rotateValue = NRMath.powWithSign(rotateValue, 3);
			
			System.out.println(gyroCorrection.getAngleError().get(Angle.Unit.DEGREE));
			
			if (Math.abs(rotateValue) < 0.05 && Math.abs(moveValue) > 0.1) {
				rotateValue = gyroCorrection.getTurnValue(Drive.getInstance().kP_thetaOneD);
			} else {
				gyroCorrection.clearInitialValue();
			}
			Drive.getInstance().arcadeDrive(moveValue * OI.getInstance().getDriveSpeedMultiplier(), rotateValue * OI.getInstance().getDriveSpeedMultiplier(), hValue * OI.getInstance().getDriveSpeedMultiplier());
			break;
		
		case tankDrive:
			double left = OI.getInstance().getTankLeftValue();
			double right = OI.getInstance().getTankRightValue();
			double hDrive = OI.getInstance().getTankHValue();
			
			left = (Math.abs(left) + Math.abs(right)) / 2 * Math.signum(left);
			right = (Math.abs(left) + Math.abs(right)) / 2 * Math.signum(right);
			
			right = NRMath.powWithSign(right, 3);
			left = NRMath.powWithSign(left, 3);
			Drive.getInstance().tankDrive(OI.getInstance().getDriveSpeedMultiplier() * left, -OI.getInstance().getDriveSpeedMultiplier() * right, OI.getInstance().getDriveSpeedMultiplier() * hDrive);
			break;
			
		case arcadeNegInertia:
			
			
			break;
		}
	}

	@Override
	public boolean shouldSwitchToJoystick() {
		if(OI.getInstance().DriveMode.getSelected() == Drive.DriveMode.arcadeDrive || OI.getInstance().DriveMode.getSelected() == Drive.DriveMode.arcadeNegInertia) {
			return OI.getInstance().getArcadeMoveValue() != 0 || OI.getInstance().getArcadeTurnValue() != 0 || OI.getInstance().getArcadeHValue() != 0;
		} else {
			return OI.getInstance().getTankLeftValue() != 0 || OI.getInstance().getTankRightValue() != 0 || OI.getInstance().getTankHValue() != 0;
		} 
	}

	@Override
	public long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}

}