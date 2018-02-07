package edu.nr.robotics.testSequence;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.climber.Climber;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandler;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;

public class SystemTestSequenceCommand extends NRCommand {
	
	private double climberCounter = 0;
	private double cubeHandlerCounter = 0;
	private double elevatorCounter = 0;
	private double elevatorShooterCounter = 0;
	private double intakeElevatorCounter = 0;
	private double intakeRollersCounter = 0;
	
	public final double MIN_DETECTION_CURRENT = 0.125;

	private boolean isIntakeElevRevLimitSwitchHit = false;
	
	private double climberCurrent = 0;
	private double cubeHandlerCurrent = 0;
	private double elevatorCurrent = 0;
	private double elevatorShooterCurrent = 0;
	private double intakeElevatorCurrent = 0;
	private double intakeRollersCurrent = 0;
	
	private double climberCurrentAve = 0;
	private double cubeHandlerCurrentAve = 0;
	private double elevatorCurrentAve = 0;
	private double elevatorShooterCurrentAve = 0;
	private double intakeElevatorCurrentAve = 0;
	private double intakeRollersCurrentAve = 0;
	
	private final double CLIMBER_STRESS_CURRENT = 0;
	private final double CUBE_HANDLER_STRESS_CURRENT = 0;
	private final double ELEVATOR_STRESS_CURRENT = 0;
	private final double ELEVATOR_SHOOTER_STRESS_CURRENT = 0;
	private final double INTAKE_ELEVATOR_STRESS_CURRENT = 0;
	private final double INTAKE_ROLLERS_STRESS_CURRENT = 0;
	
	private final double CLIMBER_LOW_CURRENT = 0;
	private final double CUBE_HANDLER_LOW_CURRENT = 0;
	private final double ELEVATOR_LOW_CURRENT = 0;
	private final double ELEVATOR_SHOOTER_LOW_CURRENT = 0;
	private final double INTAKE_ELEVATOR_LOW_CURRENT = 0;
	private final double INTAKE_ROLLERS_LOW_CURRENT = 0;
	
	public SystemTestSequenceCommand() {
		
	}
	
	@Override
	public void onExecute() {
		
		if (Climber.getInstance().getCurrent() > MIN_DETECTION_CURRENT) {
			climberCurrent += Climber.getInstance().getCurrent();
			climberCounter++;
		}
		if (CubeHandler.getInstance().getCurrent() > MIN_DETECTION_CURRENT) {
			cubeHandlerCurrent += CubeHandler.getInstance().getCurrent();
			cubeHandlerCounter++;
		}
		if (Elevator.getInstance().getCurrent() > MIN_DETECTION_CURRENT) {
			elevatorCurrent += Elevator.getInstance().getCurrent();
			elevatorCounter++;
		}
		if (ElevatorShooter.getInstance().getCurrent() > MIN_DETECTION_CURRENT) {
			elevatorShooterCurrent += ElevatorShooter.getInstance().getCurrent();
			elevatorShooterCounter++;
		}
		if (IntakeElevator.getInstance().getCurrent() > MIN_DETECTION_CURRENT) {
			intakeElevatorCurrent += IntakeElevator.getInstance().getCurrent();
			intakeElevatorCounter++;
		}
		if (IntakeRollers.getInstance().getCurrent() > MIN_DETECTION_CURRENT) {
			intakeRollersCurrent += IntakeRollers.getInstance().getCurrent();
			intakeRollersCounter++;
		}

		if (IntakeElevator.getInstance().isRevLimitSwitchClosed()) {
			isIntakeElevRevLimitSwitchHit = true;
		}
		
	}
	
	@Override
	public void onEnd() {
		climberCurrentAve = climberCurrent / climberCounter;
		cubeHandlerCurrentAve = cubeHandlerCurrent / cubeHandlerCounter;
		elevatorCurrentAve = elevatorCurrent / elevatorCounter;
		elevatorShooterCurrentAve = elevatorShooterCurrent / elevatorShooterCounter;
		intakeElevatorCurrentAve = intakeElevatorCurrent / intakeElevatorCounter;
		intakeRollersCurrentAve = intakeRollersCurrent / intakeRollersCounter;
		
		if (climberCurrentAve > CLIMBER_STRESS_CURRENT) {
			System.out.println("CLIMBER CURRENT TOO HIGH: " + climberCurrentAve + " A");
			
		} else if (climberCurrentAve < CLIMBER_LOW_CURRENT) {
			System.out.println("CLIMBER CURRENT TOO LOW: " + climberCurrentAve + " A");
			
		}
		if (cubeHandlerCurrentAve > CUBE_HANDLER_STRESS_CURRENT) {
			System.out.println("CUBE HANDLER CURRENT TOO HIGH: " + cubeHandlerCurrentAve + " A");
			
		} else if (cubeHandlerCurrentAve < CUBE_HANDLER_LOW_CURRENT) {
			System.out.println("CUBE HANDLER CURRENT TOO LOW: " + cubeHandlerCurrentAve + " A");
			
		}
		if (elevatorCurrentAve > ELEVATOR_STRESS_CURRENT) {
			System.out.println("ELEVATOR CURRENT TOO HIGH: " + elevatorCurrentAve + " A");
			
		} else if (elevatorCurrentAve < ELEVATOR_LOW_CURRENT) {
			System.out.println("ELEVATOR CURRENT TOO LOW: " + elevatorCurrentAve + " A");
			
		}
		if (elevatorShooterCurrentAve > ELEVATOR_SHOOTER_STRESS_CURRENT) {
			System.out.println("ELEVATOR SHOOTER CURRENT TOO HIGH: " + elevatorShooterCurrentAve + " A");
			
		} else if (elevatorShooterCurrentAve < ELEVATOR_SHOOTER_LOW_CURRENT) {
			System.out.println("ELEVATOR SHOOTER CURRENT TOO LOW: " + elevatorShooterCurrentAve + " A");
			
		}
		if (intakeElevatorCurrentAve > INTAKE_ELEVATOR_STRESS_CURRENT) {
			System.out.println("INTAKE ELEVATOR CURRENT TOO HIGH: " + intakeElevatorCurrentAve + " A");
			
		} else if (intakeElevatorCurrentAve < INTAKE_ELEVATOR_LOW_CURRENT) {
			System.out.println("INTAKE ELEVATOR CURRENT TOO LOW: " + intakeElevatorCurrentAve + " A");
			
		}
		if (intakeRollersCurrentAve > INTAKE_ROLLERS_STRESS_CURRENT) {
			System.out.println("INTAKE ROLLERS CURRENT TOO HIGH: " + intakeRollersCurrentAve + " A");
			
		} else if (intakeRollersCurrentAve < INTAKE_ROLLERS_LOW_CURRENT) {
			System.out.println("INTAKE ROLLERS CURRENT TOO LOW: " + intakeRollersCurrentAve + " A");
			
		}
		if (!Elevator.getInstance().isRevLimitSwitchClosed()) {
			System.out.println("ELEVATOR REV LIMIT SWITCH NOT CLOSED!");
		}
		if (!isIntakeElevRevLimitSwitchHit) {
			System.out.println("INTAKE ELEVATOR REV LIMIT SWITCH NEVER CLOSED!");
		}
		if (!IntakeElevator.getInstance().isFwdLimitSwitchClosed()) {
			System.out.println("INTAKE ELEVATOR FWD LIMIT SWITCH NOT CLOSED!");
		}
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}
}
