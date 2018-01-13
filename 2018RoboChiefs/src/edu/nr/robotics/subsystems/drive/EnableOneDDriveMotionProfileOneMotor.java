package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerHDriveMain;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotorHDrive;
import edu.nr.lib.units.Distance;
import edu.wpi.first.wpilibj.PIDSourceType;

public class EnableOneDDriveMotionProfileOneMotor extends NRCommand {
	
	Distance distanceToDrive = Distance.ZERO;
	
	private final Distance END_THRESHOLD = new Distance(0.3, Distance.Unit.INCH);
	
	public EnableOneDDriveMotionProfileOneMotor(Distance y) {
		super(Drive.getInstance());
		distanceToDrive = y;
	}

	@Override
	public void onStart() {
		Drive.getInstance().enableOneDProfilerOneMotorH(distanceToDrive);
	}
	
	@Override
	public void onExecute() {
		//TODO: Change from left/right to H
		//System.out.println("1D Profiler enabled: " + Drive.getInstance().isOneDProfilerEnabled());
		Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
		//SmartDashboard.putString("Motion Profiler V Left", Drive.getInstance().pidGetLeft() + ":" + OneDimensionalMotionProfilerTwoMotorHDrive.velocityGoal);
		//SmartDashboard.putString("Motion Profiler V Right", Drive.getInstance().pidGetRight() + ":" + OneDimensionalMotionProfilerTwoMotorHDrive.velocityGoal);
		Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
		//SmartDashboard.putString("Motion Profiler X Left", new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotorHDrive.positionGoal + OneDimensionalMotionProfilerTwoMotorHDrive.initialPositionLeft, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotorHDrive.errorLeft, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));
		//SmartDashboard.putString("Motion Profiler X Right", new Distance(Drive.getInstance().pidGetRight(), Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotorHDrive.positionGoal + OneDimensionalMotionProfilerTwoMotorHDrive.initialPositionRight, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerTwoMotorHDrive.errorRight, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));
	
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
			&& Math.abs(((Math.abs(Drive.getInstance().getHPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H) - OneDimensionalMotionProfilerHDriveMain.initialPositionH)) - Math.abs(OneDimensionalMotionProfilerHDriveMain.posPoints.get(OneDimensionalMotionProfilerTwoMotorHDrive.posPoints.size() - 1)))) < END_THRESHOLD.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H);
			
		return finished;
	}
	
}
