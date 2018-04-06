package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.gyro.Pigeon;
import edu.nr.lib.units.Angle;

public class TurnToAngleCommand extends NRCommand {

	private Angle initialAngle;	
	private GyroCorrection gyro;
	private Angle angleGoal;
	private Angle angleToTurn;
	
	public TurnToAngleCommand(Angle angleGoal) {
		this.angleGoal = angleGoal;
	}
	
	@Override
	protected void onStart() {
		
		if (Pigeon.getPigeon(Drive.getInstance().getPigeonTalon()).getYaw().signum() <= 0) {
			angleToTurn = new Angle(-angleGoal.get(Angle.Unit.DEGREE) - ((Pigeon.getPigeon(Drive.getInstance().getPigeonTalon()).getYaw().get(Angle.Unit.DEGREE)) % 360), Angle.Unit.DEGREE).negate();
		} else {
			angleToTurn = new Angle(angleGoal.get(Angle.Unit.DEGREE) - ((Pigeon.getPigeon(Drive.getInstance().getPigeonTalon()).getYaw().get(Angle.Unit.DEGREE)) % 360), Angle.Unit.DEGREE).negate();
		}
		
		System.out.println(angleToTurn.get(Angle.Unit.DEGREE));
		
		gyro = new GyroCorrection(angleToTurn, Drive.MAX_PROFILE_TURN_PERCENT);
		Drive.getInstance().pidWrite(0, 0, 0);
		initialAngle = gyro.getAngleError().sub(angleToTurn);
	}
	
	@Override
	public void onExecute() {
		
		double headingAdjustment = gyro.getTurnValue(true);
		if (Math.abs(headingAdjustment) < Drive.MIN_PROFILE_TURN_PERCENT) {
			headingAdjustment = Drive.MIN_PROFILE_TURN_PERCENT * Math.signum(headingAdjustment);
		}
		
		double outputLeft, outputRight;
		
		outputLeft = -headingAdjustment;
		outputRight = headingAdjustment;
		
		Drive.getInstance().pidWrite(outputLeft, outputRight, 0);
		
	}
	
	@Override
	public void onEnd() {
		Drive.getInstance().disable();
	}
	
	@Override
	public boolean isFinishedNR() {
		
		boolean finished = Drive.getInstance().getLeftVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD)
				&& Drive.getInstance().getRightVelocity().lessThan(Drive.PROFILE_END_SPEED_THRESHOLD) && 
				(initialAngle.sub(gyro.getAngleError())).abs().lessThan(Drive.DRIVE_ANGLE_THRESHOLD);
		return finished;
	}
}