package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.GyroCorrection;
import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.robotics.OI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
				
				moveValue = NRMath.powWithSign(moveValue, 3);
				rotateValue = NRMath.powWithSign(rotateValue, 3);
				
				if (Math.abs(rotateValue) < 0.05 && Math.abs(moveValue) > 0.1) {
					rotateValue = gyroCorrection.getTurnValue(0.04);
				} else {
					gyroCorrection.clearInitialValue();
				}
				Drive.getInstance().arcadeDrive(moveValue * OI.getInstance().getDriveSpeedMultiplier(), rotateValue * OI.getInstance().getDriveSpeedMultiplier());
		
		case tankDrive:
			double left = OI.getInstance().getTankLeftValue();
			double right = OI.getInstance().getTankRightValue();
			
			left = (Math.abs(left) + Math.abs(right)) / 2 * Math.signum(left);
			right = (Math.abs(left) + Math.abs(right)) / 2 * Math.signum(right);
			
			right = NRMath.powWithSign(right, 3);
			left = NRMath.powWithSign(left, 3);
			Drive.getInstance().tankDrive(OI.getInstance().getDriveSpeedMultiplier() * left, -OI.getInstance().getDriveSpeedMultiplier() * right);
			
		case cheesyDrive: //TODO: Find out what to do for cheesy drive
			
			break;
		}
	}

	@Override
	public boolean shouldSwitchToJoystick() {
		if(OI.getInstance().DriveMode.getSelected() == Drive.DriveMode.arcadeDrive) {
			return OI.getInstance().getArcadeMoveValue() != 0 || OI.getInstance().getArcadeTurnValue() != 0;
		} else if (OI.getInstance().DriveMode.getSelected() == Drive.DriveMode.tankDrive) {
			return OI.getInstance().getTankLeftValue() != 0 || OI.getInstance().getTankRightValue() != 0;
		} else {
			return (Boolean) null; //TODO: Find out what to do for cheesydrive
		}
	}

	@Override
	public long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}

}