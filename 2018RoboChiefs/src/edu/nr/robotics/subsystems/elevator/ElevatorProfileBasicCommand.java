package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerBasic;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorProfileBasicCommand extends NRCommand {
	
	
	Distance initialPosition;
	Distance tempPosition = Distance.ZERO;
	
	Distance position;
	double maxVelPercent;
	double maxAccelPercent;
	
	public ElevatorProfileBasicCommand(Distance position, double maxVelPercent, double maxAccelPercent) {
		super(Elevator.getInstance());
		this.position = position;
		this.maxVelPercent = maxVelPercent;
		this.maxAccelPercent = maxAccelPercent;
	}
	
	@Override
	public void onStart() {
		initialPosition = Elevator.getInstance().getPosition();
		Elevator.getInstance().enableMotionProfiler(position.sub(initialPosition), maxVelPercent, maxAccelPercent);
	}
	
	@Override
	public void onExecute() {
		if (EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {	
			Elevator.getInstance().setPIDSourceType(PIDSourceType.kRate);
			SmartDashboard.putString("Motion Profiler V", Elevator.getInstance().pidGet() + ":" + OneDimensionalMotionProfilerBasic.velocityGoal);
			Elevator.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
			SmartDashboard.putString("Motion Profiler X", new Distance(Elevator.getInstance().pidGet(), Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerBasic.positionGoal + OneDimensionalMotionProfilerBasic.initialPosition, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerBasic.error, Distance.Unit.MAGNETIC_ENCODER_TICK).get(Distance.Unit.INCH));
	
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