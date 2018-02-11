package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerBasic;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorProfileBasicSmartDashboardCommand extends NRCommand {
	
	
	Distance initialPosition;
	Distance tempPosition = Distance.ZERO;
	
	Distance position;
	double maxVelPercent;
	double maxAccelPercent;
		
	public ElevatorProfileBasicSmartDashboardCommand() {
		super(Elevator.getInstance());
		this.position = Elevator.profileDeltaPos;
		maxVelPercent = Elevator.MAX_SPEED_ELEVATOR.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND);
		maxAccelPercent = Elevator.MAX_ACCEL_ELEVATOR.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND);
	}
	
	@Override
	public void onStart() {
		initialPosition = Elevator.getInstance().getPosition();
		Elevator.getInstance().enableMotionProfiler(position.sub(initialPosition), maxVelPercent, maxAccelPercent);
	}
	
	@Override
	public void onExecute() {
		if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {	
			Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
			SmartDashboard.putString("Motion Profiler V", Drive.getInstance().pidGetH() + ":" + OneDimensionalMotionProfilerBasic.velocityGoal);
			Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
			SmartDashboard.putString("Motion Profiler X", new Distance(Drive.getInstance().pidGetLeft(), Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerBasic.positionGoal + OneDimensionalMotionProfilerBasic.initialPosition, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerBasic.error, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));
	
		}
	}
	
	@Override
	public void onEnd() {
		Elevator.getInstance().disableProfiler();
		Elevator.getInstance().setMotorSpeedPercent(0);
	}
	
	@Override
	public boolean isFinishedNR() {
		boolean finished = (Elevator.getInstance().getHistoricalPosition(Elevator.PROFILE_DELTA_TIME_THRESHOLD_ELEVATOR)
				.sub(Elevator.getInstance().getPosition())).abs().lessThan(Elevator.PROFILE_DELTA_POS_THRESHOLD_ELEVATOR)
				&& (Elevator.getInstance().getHistoricalPosition(Elevator.PROFILE_DELTA_TIME_THRESHOLD_ELEVATOR.mul(2))
						.sub(Elevator.getInstance().getPosition())).abs().lessThan(Elevator.PROFILE_DELTA_POS_THRESHOLD_ELEVATOR)
				&& (initialPosition.add(position).sub(Elevator.getInstance().getPosition())).abs().lessThan(Elevator.PROFILE_END_POS_THRESHOLD_ELEVATOR);
		return finished;
	}

}
