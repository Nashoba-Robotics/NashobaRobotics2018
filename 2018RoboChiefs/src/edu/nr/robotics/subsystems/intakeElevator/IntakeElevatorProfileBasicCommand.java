package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerBasic;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeElevatorProfileBasicCommand extends NRCommand {
	
	
	Distance initialPosition;
	Distance tempPosition = Distance.ZERO;
	
	Distance position;
	double maxVelPercent;
	double maxAccelPercent;
	
	public IntakeElevatorProfileBasicCommand(Distance position, double maxVelPercent, double maxAccelPercent) {
		super(IntakeElevator.getInstance());
		this.position = position;
		this.maxVelPercent = maxVelPercent;
		this.maxAccelPercent = maxAccelPercent;
	}
	
	@Override
	public void onStart() {
		initialPosition = IntakeElevator.getInstance().getPosition();
		IntakeElevator.getInstance().enableMotionProfiler(position.sub(initialPosition), maxVelPercent, maxAccelPercent);
	}
	
	@Override
	public void onExecute() {
		if (EnabledSubsystems.INTAKE_ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {	
			IntakeElevator.getInstance().setPIDSourceType(PIDSourceType.kRate);
			SmartDashboard.putString("Intake Elevator Motion Profiler V", IntakeElevator.getInstance().pidGet() + ":" + IntakeElevator.getInstance().basicProfiler.velocityGoal);
			IntakeElevator.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
			//SmartDashboard.putString("Elevator Motion Profiler X", new Distance(Elevator.getInstance().pidGet(), Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerBasic.positionGoal + OneDimensionalMotionProfilerBasic.initialPosition, Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerBasic.error, Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV).get(Distance.Unit.INCH));	
		}
	}
	
	@Override
	public void onEnd() {
		IntakeElevator.getInstance().disableProfiler();
		IntakeElevator.getInstance().setMotorSpeedPercent(0);
	}
	
	@Override
	public boolean isFinishedNR() {
		boolean finished = IntakeElevator.getInstance().basicProfiler.loopIteration > IntakeElevator.getInstance().basicProfiler.posPoints.size();
		return finished;
	}

}
