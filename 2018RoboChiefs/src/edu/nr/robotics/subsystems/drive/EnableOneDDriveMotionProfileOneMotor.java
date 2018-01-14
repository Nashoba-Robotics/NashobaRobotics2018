package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerHDriveMain;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotorHDrive;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.PIDSourceType;

public class EnableOneDDriveMotionProfileOneMotor extends NRCommand {
	
	Distance distanceToDrive = Distance.ZERO;
	double percent = 0;
	
	public EnableOneDDriveMotionProfileOneMotor(Distance y) {
		this(y, Drive.PROFILE_DRIVE_PERCENT);
	}
	
	public EnableOneDDriveMotionProfileOneMotor(Distance y, double percent) {
		super(Drive.getInstance());
		distanceToDrive = y;
		this.percent = percent;
	}

	@Override
	public void onStart() {
		Drive.getInstance().enableOneDProfilerOneMotorH(distanceToDrive);
	}
	
	@Override
	public void onExecute() {
		//System.out.println("1D Profiler enabled: " + Drive.getInstance().isOneDProfilerEnabled());
		Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
		//SmartDashboard.putString("Motion Profiler V H", Drive.getInstance().pidGetH() + ":" + OneDimensionalMotionProfilerHDriveMain.velocityGoal);
		Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
		//SmartDashboard.putString("Motion Profiler X H", new Distance(Drive.getInstance().pidGetH(), Distance.Unit.MAGNETIC_ENCODER_TICK_H).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerHDriveMain.positionGoal + OneDimensionalMotionProfilerHDriveMain.initialPositionH, Distance.Unit.MAGNETIC_ENCODER_TICK_H).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerHDriveMain.errorH, Distance.Unit.MAGNETIC_ENCODER_TICK_H).get(Distance.Unit.INCH));
	
	}
	
	@Override
	public void onEnd() {
		Drive.getInstance().disableOneDProfilerOneMotorH();
	}
	
	@Override
	public boolean isFinishedNR() {
				
		
		boolean finished = (Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getHPosition())).abs()
			.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
			&& (Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getHPosition())).abs()
			.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
			&& Math.abs(((Math.abs(Drive.getInstance().getHPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H) - OneDimensionalMotionProfilerHDriveMain.initialPositionH)) - Math.abs(OneDimensionalMotionProfilerHDriveMain.posPoints.get(OneDimensionalMotionProfilerTwoMotorHDrive.posPoints.size() - 1)))) < Drive.END_THRESHOLD.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H);
			
		return finished;
	}
	
}
