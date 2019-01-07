package edu.nr.lib.gyro;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.Gyro.ChosenGyro;
import edu.nr.robotics.subsystems.drive.Drive;

public class ResetGyroCommand extends NRCommand {

	public ResetGyroCommand() {
		
	}

	@Override
	protected void onStart() {
		Pigeon.getPigeon(Drive.getInstance().getPigeonTalon()).reset();
		
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
