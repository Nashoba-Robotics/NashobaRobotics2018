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
	
	private Distance position;
	private double maxVelPercent;
	private double maxAccelPercent;
		
	public ElevatorProfileBasicSmartDashboardCommand() {
		super(Elevator.getInstance());
	}
	
	@Override
	public void onStart() {
		
		this.position = Elevator.profilePos;
		this.maxVelPercent = Elevator.PROFILE_VEL_PERCENT_ELEVATOR;
		this.maxAccelPercent = Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR;
		
		initialPosition = Elevator.getInstance().getPosition();
		Elevator.getInstance().enableMotionProfiler(position.sub(initialPosition), maxVelPercent, maxAccelPercent);
	}
	
	@Override
	public void onExecute() {
		if (EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {	
			Drive.getInstance().setPIDSourceType(PIDSourceType.kRate);
			SmartDashboard.putString("Elevator Motion Profiler V", Elevator.getInstance().pidGet() + ":" + Elevator.getInstance().basicProfiler.velocityGoal);
			Drive.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
			//SmartDashboard.putString("Elevator Motion Profiler X", new Distance(Elevator.getInstance().pidGet(), Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerBasic.positionGoal + OneDimensionalMotionProfilerBasic.initialPosition, Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerBasic.error, Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV).get(Distance.Unit.INCH));
	
		}
	}
	
	@Override
	public void onEnd() {
		Elevator.getInstance().disableProfiler();
		Elevator.getInstance().setMotorPercentRaw(Elevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP);
	}
	
	@Override
	public boolean isFinishedNR() {
		boolean finished = Elevator.getInstance().basicProfiler.loopIteration > Elevator.getInstance().basicProfiler.posPoints.size() - 2;
		return finished;
	}

}
