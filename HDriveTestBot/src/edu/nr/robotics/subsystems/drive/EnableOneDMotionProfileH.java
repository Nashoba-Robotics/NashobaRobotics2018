package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerHDriveMain;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerHDriveMain;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableOneDMotionProfileH extends NRCommand {
	
	Distance initialLeftPosition;
	Distance initialRightPosition;
	Distance initialHPosition;
	Distance tempLeftPosition = Distance.ZERO;
	Distance tempRightPosition = Distance.ZERO;
	
	private final Distance END_THRESHOLD = new Distance(0.3, Distance.Unit.INCH);
	
	public EnableOneDMotionProfileH() {
		super(Drive.getInstance());
	}
	
	@Override
	public void onStart() {
		Drive.getInstance().enableOneDProfilerH(Drive.oneDProfileHMain);
		initialLeftPosition = Drive.getInstance().getLeftDistance();
		initialRightPosition = Drive.getInstance().getRightDistance();
	}
	
	@Override
	public void onExecute() {
		//System.out.println("1D Profiler enabled: " + Drive.getInstance().isOneDProfilerEnabled());
		Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
		SmartDashboard.putString("Motion Profiler V H", Drive.getInstance().pidGetLeft() + ":" + OneDimensionalMotionProfilerHDriveMain.velocityGoal);
		Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
		SmartDashboard.putString("Motion Profiler X H", new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK_H).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerHDriveMain.positionGoal + OneDimensionalMotionProfilerHDriveMain.initialPositionH, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerHDriveMain.errorH, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));
	}

	@Override
	public void onEnd() {
		Drive.getInstance().disableOneDProfilerH();
	}
	
	@Override
	public boolean isFinishedNR() {
				
		boolean finished = (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getLeftDistance())).abs()
			.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
			&& (Drive.getInstance().getHistoricalLeftPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getLeftDistance())).abs()
			.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
			&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD).sub(Drive.getInstance().getRightDistance())).abs()
			.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
			&& (Drive.getInstance().getHistoricalRightPosition(Drive.PROFILE_TIME_THRESHOLD.mul(2)).sub(Drive.getInstance().getRightDistance())).abs()
			.lessThan(Drive.PROFILE_POSITION_THRESHOLD)
			&& Math.abs(((Math.abs(Drive.getInstance().getLeftDistance().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H) - OneDimensionalMotionProfilerHDriveMain.initialPositionLeft)) - Math.abs(OneDimensionalMotionProfilerHDriveMain.posPoints.get(OneDimensionalMotionProfilerHDriveMain.posPoints.size() - 1)))) < END_THRESHOLD.get(Distance.Unit.MAGNETIC_ENCODER_TICK);
			
		return finished;
	}

}
