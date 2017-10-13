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
			
		case cheesyDrive:
			
			break;
		}
	}

	@Override
	public boolean shouldSwitchToJoystick() {
		if(OI.driveMode == Drive.DriveMode.arcadeDrive) {
			return OI.getInstance().getArcadeMoveValue() != 0 || OI.getInstance().getArcadeTurnValue() != 0;
		} else {
			return OI.getInstance().getTankLeftValue() != 0 || OI.getInstance().getTankRightValue() != 0;
		}
	}

	@Override
	public long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}

}