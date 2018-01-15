package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerHDriveMain;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerHDriveMain;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableOneDMotionProfileH extends NRCommand {
	
	Distance initialHPosition;
	
	private final Distance END_THRESHOLD = new Distance(3, Distance.Unit.INCH);
	
	public EnableOneDMotionProfileH() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		Drive.getInstance().enableOneDProfilerH(Drive.oneDProfileHMain);
	}
	
	@Override
	public void onExecute() {
		Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
		SmartDashboard.putString("Motion Profiler V H", Drive.getInstance().pidGetH() + ":" + OneDimensionalMotionProfilerHDriveMain.velocityGoal);
		Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
		SmartDashboard.putString("Motion Profiler X H", new Distance(Drive.getInstance().pidGetH(), Distance.Unit.MAGNETIC_ENCODER_TICK_H).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerHDriveMain.positionGoal + OneDimensionalMotionProfilerHDriveMain.initialPositionH, Distance.Unit.MAGNETIC_ENCODER_TICK_H).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerHDriveMain.errorH, Distance.Unit.MAGNETIC_ENCODER_TICK_H).get(Distance.Unit.INCH));
	}

	@Override
	public void onEnd() {
		Drive.getInstance().disableOneDProfilerH();
	}
	
	@Override
	public boolean isFinishedNR() {

		boolean finished = (Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getHDistance())).abs()
			.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
			&& (Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getHDistance())).abs()
			.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
			&& Math.abs(((Math.abs(Drive.getInstance().getHDistance().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H) - OneDimensionalMotionProfilerHDriveMain.initialPositionH)) - Math.abs(OneDimensionalMotionProfilerHDriveMain.posPoints.get(OneDimensionalMotionProfilerHDriveMain.posPoints.size() - 1)))) < END_THRESHOLD.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H);
		
		System.out.println(Drive.getInstance().getHistoricalHPosition(Drive.PROFILE_TIME_THRESHOLD));
		
		return finished;
	}

}
