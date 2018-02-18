package edu.nr.robotics.testSequence;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.climber.Climber;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;

public class SystemTestSequenceCommand extends NRCommand {
	
	private double climberCounter = 0;
	private double elevatorCounter = 0;
	private double elevatorShooterCounter = 0;
	private double intakeElevatorCounter = 0;
	private double intakeRollersLeftCounter = 0;
	private double intakeRollersRightCounter = 0;
	
	public final double MIN_DETECTION_CURRENT = 0.125;

	private boolean isIntakeElevRevLimitSwitchHit = false;
	
	private double climberCurrent = 0;
	private double cubeHandlerCurrent = 0;
	private double elevatorCurrent = 0;
	private double elevatorShooterCurrent = 0;
	private double intakeElevatorCurrent = 0;
	private double intakeRollersLeftCurrent = 0;
	private double intakeRollersRightCurrent = 0;
	
	private double climberCurrentAve = 0;
	private double elevatorCurrentAve = 0;
	private double elevatorShooterCurrentAve = 0;
	private double intakeElevatorCurrentAve = 0;
	private double intakeRollersCurrentLeftAve = 0;
	private double intakeRollersCurrentRightAve = 0;
	
	private final double CLIMBER_STRESS_CURRENT = 0;
	private final double ELEVATOR_STRESS_CURRENT = 0;
	private final double ELEVATOR_SHOOTER_STRESS_CURRENT = 0;
	private final double INTAKE_ELEVATOR_STRESS_CURRENT = 0;
	private final double INTAKE_ROLLERS_STRESS_CURRENT = 0;
	
	private final double CLIMBER_LOW_CURRENT = 0;
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
		if (Elevator.getInstance().getMasterCurrent() > MIN_DETECTION_CURRENT) {
			elevatorCurrent += Elevator.getInstance().getMasterCurrent();
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
		if (IntakeRollers.getInstance().getCurrentLeft() > MIN_DETECTION_CURRENT) {
			intakeRollersLeftCurrent += IntakeRollers.getInstance().getCurrentLeft();
			intakeRollersLeftCounter++;
		}
		if (IntakeRollers.getInstance().getCurrentRight() > MIN_DETECTION_CURRENT) {
			intakeRollersRightCurrent += IntakeRollers.getInstance().getCurrentRight();
			intakeRollersRightCounter++;
		}
		if (IntakeElevator.getInstance().isRevLimitSwitchClosed()) {
			isIntakeElevRevLimitSwitchHit = true;
		}
		
	}
	
	@Override
	public void onEnd() {
		
		climberCurrentAve = climberCurrent / climberCounter;
		elevatorCurrentAve = elevatorCurrent / elevatorCounter;
		elevatorShooterCurrentAve = elevatorShooterCurrent / elevatorShooterCounter;
		intakeElevatorCurrentAve = intakeElevatorCurrent / intakeElevatorCounter;
		intakeRollersCurrentLeftAve = intakeRollersLeftCurrent / intakeRollersLeftCounter;
		
		if (climberCurrentAve > CLIMBER_STRESS_CURRENT) {
			System.out.println("CLIMBER CURRENT TOO HIGH: " + climberCurrentAve + " A");	
		} else if (climberCurrentAve < CLIMBER_LOW_CURRENT) {
			System.out.println("CLIMBER CURRENT TOO LOW: " + climberCurrentAve + " A");	
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
		
		if (intakeRollersCurrentLeftAve > INTAKE_ROLLERS_STRESS_CURRENT) {
			System.out.println("INTAKE ROLLERS LEFT CURRENT TOO HIGH: " + intakeRollersCurrentLeftAve + " A");	
		} else if (intakeRollersCurrentLeftAve < INTAKE_ROLLERS_LOW_CURRENT) {
			System.out.println("INTAKE ROLLERS LEFT CURRENT TOO LOW: " + intakeRollersCurrentLeftAve + " A");
		}
		
		if (intakeRollersCurrentRightAve > INTAKE_ROLLERS_STRESS_CURRENT) {
			System.out.println("INTAKE ROLLERS RIGHT CURRENT TOO HIGH: " + intakeRollersCurrentRightAve + " A");
		} else if (intakeRollersCurrentRightAve < INTAKE_ROLLERS_LOW_CURRENT) {
			System.out.println("INTAKE ROLLERS RIGHT CURRENT TOO LOW: " + intakeRollersCurrentRightAve + " A");
		
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
