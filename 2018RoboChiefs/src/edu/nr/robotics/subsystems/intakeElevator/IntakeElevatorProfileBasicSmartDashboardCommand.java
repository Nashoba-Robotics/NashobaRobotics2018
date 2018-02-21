package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerBasic;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeElevatorProfileBasicSmartDashboardCommand extends NRCommand {
	
	
	Distance initialPosition;
	Distance tempPosition = Distance.ZERO;
	
	Distance position;
	double maxVelPercent;
	double maxAccelPercent;
	
	public IntakeElevatorProfileBasicSmartDashboardCommand() {
		super(IntakeElevator.getInstance());
		this.position = IntakeElevator.profileDeltaPos;
		this.maxVelPercent = IntakeElevator.PROFILE_VEL_PERCENT_INTAKE_ELEVATOR;
		this.maxAccelPercent = IntakeElevator.PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR;
	}
	
	@Override
	public void onStart() {
		this.position = IntakeElevator.profileDeltaPos;
		System.out.println("delta pos: " + IntakeElevator.profileDeltaPos.get(Distance.Unit.INCH));
		initialPosition = IntakeElevator.getInstance().getPosition();
		IntakeElevator.getInstance().enableMotionProfiler(position.sub(initialPosition), maxVelPercent, maxAccelPercent);
		System.out.println("position: " + position.sub(initialPosition).get(Distance.Unit.INCH) + " maxVelPercent: " + maxVelPercent + " maxAccelPercent: " + maxAccelPercent);
	}
	
	@Override
	public void onExecute() {
		if (EnabledSubsystems.INTAKE_ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {	
			IntakeElevator.getInstance().setPIDSourceType(PIDSourceType.kRate);
			SmartDashboard.putString("Intake Elevator Motion Profiler V", IntakeElevator.getInstance().pidGet() + ":" + IntakeElevator.getInstance().basicProfiler.velocityGoal);
			IntakeElevator.getInstance().setPIDSourceType(PIDSourceType.kDisplacement);
			//SmartDashboard.putString("Intake Elevator Motion Profiler X", new Distance(IntakeElevator.getInstance().pidGet(), Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerBasic.positionGoal + OneDimensionalMotionProfilerBasic.initialPosition, Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV).get(Distance.Unit.INCH) + ":" + new Distance(OneDimensionalMotionProfilerBasic.error, Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV).get(Distance.Unit.INCH));	
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
