package edu.nr.robotics.testSequence;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.climber.Climber;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;

public class SystemTestSequenceCommand extends NRCommand {
	
	private Distance initIntakeElevatorPos = Distance.ZERO;
	private Distance initElevatorPos = Distance.ZERO;
	
	private boolean intakeElevEncWorks = false;
	private boolean elevEncWorks = false;
	
	private Distance encDifThreshold = new Distance(2, Distance.Unit.INCH);
	
	private double climberCounter = 0;
	private double elevatorCounter = 0;
	private double elevatorShooterCounter = 0;
	private double intakeElevatorCounter = 0;
	private double intakeRollersLeftCounter = 0;
	private double intakeRollersRightCounter = 0;
	
	public final double MIN_DETECTION_CURRENT = 0.125;

	private boolean isIntakeElevRevLimitSwitchHit = false;
	
	private double climberCurrent = 0;
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
	protected void onStart() {
		initIntakeElevatorPos = IntakeElevator.getInstance().getPosition();
		initElevatorPos = Elevator.getInstance().getPosition();
		intakeElevEncWorks = false;
		elevEncWorks = false;
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
		if (!IntakeElevator.getInstance().isRevLimitSwitchClosed() && !intakeElevEncWorks && (IntakeElevator.getInstance().getPosition().sub(initIntakeElevatorPos)).abs().greaterThan(encDifThreshold)) {
			intakeElevEncWorks = true;
		}
		if (!elevEncWorks && (Elevator.getInstance().getPosition().sub(initElevatorPos)).abs().greaterThan(encDifThreshold)) {
			elevEncWorks = true;
		}
		
	}
	
	@Override
	public void onEnd() {
		
		climberCurrentAve = climberCurrent / climberCounter;
		elevatorCurrentAve = elevatorCurrent / elevatorCounter;
		elevatorShooterCurrentAve = elevatorShooterCurrent / elevatorShooterCounter;
		intakeElevatorCurrentAve = intakeElevatorCurrent / intakeElevatorCounter;
		intakeRollersCurrentLeftAve = intakeRollersLeftCurrent / intakeRollersLeftCounter;
		intakeRollersCurrentRightAve = intakeRollersRightCurrent / intakeRollersRightCounter;
		
		if (climberCurrentAve > CLIMBER_STRESS_CURRENT) {
			System.out.println("CLIMBER CURRENT TOO HIGH: " + Math.round(climberCurrentAve) + " A");	
		} else if (climberCurrentAve < CLIMBER_LOW_CURRENT) {
			System.out.println("CLIMBER CURRENT TOO LOW: " + Math.round(climberCurrentAve) + " A");	
		} else {
			System.out.println("Climber Normal: " + Math.round(climberCurrentAve) + " A");
		}
		
		if (elevatorCurrentAve > ELEVATOR_STRESS_CURRENT) {
			System.out.println("ELEVATOR CURRENT TOO HIGH: " + Math.round(elevatorCurrentAve) + " A");	
		} else if (elevatorCurrentAve < ELEVATOR_LOW_CURRENT) {
			System.out.println("ELEVATOR CURRENT TOO LOW: " + Math.round(elevatorCurrentAve) + " A");	
		} else {
			System.out.println("Elevator Normal: " + Math.round(elevatorCurrentAve) + " A");
		}
		
		if (elevatorShooterCurrentAve > ELEVATOR_SHOOTER_STRESS_CURRENT) {
			System.out.println("ELEVATOR SHOOTER CURRENT TOO HIGH: " + Math.round(elevatorShooterCurrentAve) + " A");
		} else if (elevatorShooterCurrentAve < ELEVATOR_SHOOTER_LOW_CURRENT) {
			System.out.println("ELEVATOR SHOOTER CURRENT TOO LOW: " + Math.round(elevatorShooterCurrentAve) + " A");	
		} else {
			System.out.println("Elevator Shooter Normal: " + Math.round(elevatorShooterCurrentAve) + " A");
		}
		
		if (intakeElevatorCurrentAve > INTAKE_ELEVATOR_STRESS_CURRENT) {
			System.out.println("INTAKE ELEVATOR CURRENT TOO HIGH: " + Math.round(intakeElevatorCurrentAve) + " A");	
		} else if (intakeElevatorCurrentAve < INTAKE_ELEVATOR_LOW_CURRENT) {
			System.out.println("INTAKE ELEVATOR CURRENT TOO LOW: " + Math.round(intakeElevatorCurrentAve) + " A");	
		} else {
			System.out.println("Intake Elevator Normal: " + Math.round(intakeElevatorCurrentAve) + " A");
		}
		
		if (intakeRollersCurrentLeftAve > INTAKE_ROLLERS_STRESS_CURRENT) {
			System.out.println("INTAKE ROLLERS LEFT CURRENT TOO HIGH: " + Math.round(intakeRollersCurrentLeftAve) + " A");	
		} else if (intakeRollersCurrentLeftAve < INTAKE_ROLLERS_LOW_CURRENT) {
			System.out.println("INTAKE ROLLERS LEFT CURRENT TOO LOW: " + Math.round(intakeRollersCurrentLeftAve) + " A");
		} else {
			System.out.println("Intake Rollers Left Normal: " + Math.round(intakeRollersCurrentLeftAve) + " A");
		}
		
		if (intakeRollersCurrentRightAve > INTAKE_ROLLERS_STRESS_CURRENT) {
			System.out.println("INTAKE ROLLERS RIGHT CURRENT TOO HIGH: " + Math.round(intakeRollersCurrentRightAve) + " A");
		} else if (intakeRollersCurrentRightAve < INTAKE_ROLLERS_LOW_CURRENT) {
			System.out.println("INTAKE ROLLERS RIGHT CURRENT TOO LOW: " + Math.round(intakeRollersCurrentRightAve) + " A");
		} else {
			System.out.println("Intake Rollers Right Normal: " + Math.round(intakeRollersCurrentRightAve) + " A");
		}
		
		if (!isIntakeElevRevLimitSwitchHit) {
			System.out.println("INTAKE ELEVATOR REV LIMIT SWITCH NEVER CLOSED!");
		}
		
		if (!intakeElevEncWorks) {
			System.out.println("INTAKE ELEVATOR ENCODER ERROR!");
		}
		
		if(!elevEncWorks) {
			System.out.println("ELEVATOR ENCODER ERROR!");
		}
		
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}
}
