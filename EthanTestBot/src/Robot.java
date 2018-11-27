package edu.nr.robotics;

import edu.nr.lib.commandbased.CancelAllCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.gyro.ResetGyroCommand;
import edu.nr.lib.interfaces.Periodic;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.OI;
import edu.nr.robotics.Robot;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.AutoChoosers;
import edu.nr.robotics.auton.AutoChoosers.AllianceBlocks;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.AutoChoosers.ScaleSwitch;
import edu.nr.robotics.auton.AutoChoosers.StartPos;
import edu.nr.robotics.auton.AutoChoosers.Switch;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.automap.StartPosLeftScaleSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosLeftScaleSwitchRightCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchOtherCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchBothCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchOtherCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchRightCommand;
import edu.nr.robotics.auton.automap.StartPosRightScaleSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosRightScaleSwitchRightCommand;
import edu.nr.robotics.auton.automap.StartPosRightSwitchOtherCommand;
import edu.nr.robotics.auton.automap.StartPosRightSwitchRightCommand;
import edu.nr.robotics.multicommands.ClimbButtonCommand;
import edu.nr.robotics.multicommands.TestShootingWhileTurningSmartDashboardCommand;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.climber.ClimberCoastCommand;
import edu.nr.robotics.subsystems.drive.CSVSaverDisable;
import edu.nr.robotics.subsystems.drive.CSVSaverEnable;
import edu.nr.robotics.subsystems.drive.DriveForwardBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.DriveForwardSmartDashboardCommandH;
import edu.nr.robotics.subsystems.drive.EnableMotionProfileSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.TurnSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorDeltaPositionSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorMoveBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileSmartDashboardCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterVelocitySmartDashboardCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorDeltaPositionSmartDashboardCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorMoveBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorProfileSmartDashboardCommandGroup;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersVelocitySmartDashboardCommand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
private double prevTime = 0;
	
	private static Robot singleton;
	
	public synchronized static Robot getInstance() {
		return singleton;
	}
	@Override
	public void robotInit(){
		singleton = this;
		
		smartDashboardInit();
		autoChooserInit();
		OI.init();
		LimelightNetworkTable.getInstance().lightLED(true);
		LimelightNetworkTable.getInstance().lightLED(false);
		
		/*for (int i = 0; i < 10; i++) {
			System.out.println("Name Geoff");
		}
		System.out.println("-Erik");
		*/
	}
	//Might hypothetically need to define auto position
	public void smartDashboardInit() {
		SmartDashboard.putData(new CSVSaverEnable());
		SmartDashboard.putData(new CSVSaverDisable());
		SmartDashboard.putData(new TestShootingWhileTurningSmartDashboardCommand());
		SmartDashboard.putNumber("Auto Wait Time", 0);
		
		//Drive
		if(EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putData(new DriveForwardBasicSmartDashboardCommand());
			SmartDashboard.putData(new EnableMotionProfileSmartDashboardCommand());
			SmartDashboard.putData(new DriveForwardSmartDashboardCommandH());
			SmartDashboard.putData(new TurnSmartDashboardCommand());
		}
	}
	@Override
	public void disabledInit() {
		for (NRSubsystem subsystem : NRSubsystem.subsystems) {
			subsystem.disable();
		}
	}
	
	@Override
	public void testInit() {
		
	}

	@Override
	public void disabledPeriodic() {
		
	}
	
	/*@Override
	public void autonomousInit() {
		
		LimelightNetworkTable.getInstance().lightLED(false);
		new ResetGyroCommand().start();
		new ClimberCoastCommand(false).start();
		
		FieldData.getInstance().getFieldData();
		selectedStartPos = AutoChoosers.autoStartPosChooser.getSelected();
		selectedSwitch = AutoChoosers.autoSwitchChooser.getSelected();
		selectedScale = AutoChoosers.autoScaleChooser.getSelected();
		selectedBlocks = AutoChoosers.allianceBlockChooser.getSelected();
		selectedScaleSwitch = AutoChoosers.autoScaleSwitchChooser.getSelected();
		autoWaitTime = SmartDashboard.getNumber("Auto Wait Time", autoWaitTime);
		
		autonomousCommand = getAutoCommand();

		System.out.println("Start Pos: " + selectedStartPos + " Switch: " + selectedSwitch + " Scale: " + selectedScale);
		
		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	} Probably auto?*/
	
	@Override
	public void autonomousPeriodic() {
	
	}
	
	@Override
	public void teleopInit() {
		
		new CancelAllCommand().start();
		
		new ClimberCoastCommand(false).start();
			
		LimelightNetworkTable.getInstance().lightLED(true);
		LimelightNetworkTable.getInstance().lightLED(false);
	}
	
	
	
}
