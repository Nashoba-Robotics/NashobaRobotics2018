package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.Equation;
import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.OI;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;

public class DriveToBallCommand extends NRCommand {

    Equation dist = new Equation() {
    
        @Override
        public double getValue(double x) {
            System.out.println("dist" + (2280.64 / (10.6409 + 0.831893*x)));
            return 2280.64 / (10.6409 + 0.831893*x);
        }
    };

    Equation velPercent = new Equation() {

        @Override
        public double getValue(double x) {
            return Math.pow(x, 1.6) / 1000;
        }

    };

    double boxHeight;
    double targetDistance;
    double moveValue;
    double headingAdjustment;
    double outputLeft;
    double outputRight;

	public DriveToBallCommand() {
		super(Drive.getInstance());
	}
	
	@Override
	protected void onStart() {
		new EnableLimelightCommand(true).start();
	}
	
	@Override
	protected void onExecute() {
        
        boxHeight = LimelightNetworkTable.getInstance().getBoxHeight();
        
        if (boxHeight != 0) {
            targetDistance = dist.getValue(boxHeight);
            moveValue = velPercent.getValue(targetDistance);
         } else {
            targetDistance = 0;
            moveValue = 0;
        }

        if (moveValue > 1) {
            moveValue = 1;
        }

        moveValue *= 0.7;
 
		
		headingAdjustment = ((-Math.cos(LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.RADIAN) 
				/ ((Drive.DRIVE_STOP_ANGLE.get(Angle.Unit.DEGREE) / 90) * 3)) 
				* (1 - Drive.MIN_PROFILE_TURN_PERCENT)) + 1 + Drive.MIN_PROFILE_TURN_PERCENT) 
				* -LimelightNetworkTable.getInstance().getHorizOffset().signum();
		if (Math.abs(headingAdjustment) < Drive.MIN_PROFILE_TURN_PERCENT) {
			headingAdjustment = Drive.MIN_PROFILE_TURN_PERCENT * Math.signum(headingAdjustment);
		}
		
		outputLeft = moveValue - headingAdjustment;
		outputRight = moveValue + headingAdjustment;
		
		Drive.getInstance().setMotorSpeedInPercent(outputLeft, outputRight, 0);
	}
	
	@Override
	protected void onEnd() {
		new EnableLimelightCommand(false).start();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}
}
