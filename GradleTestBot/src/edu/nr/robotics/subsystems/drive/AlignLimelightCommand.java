package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;

public class AlignLimelightCommand extends NRCommand {
	
	private DoublePIDOutput out;
	private LimelightNetworkTable.Pipeline pipeline;
	
	public AlignLimelightCommand(DoublePIDOutput out, LimelightNetworkTable.Pipeline pipeline) {
		this.out = out;
		this.pipeline = pipeline;
	}
	
	@Override
	public void onStart() {
		out.pidWrite(0, 0);
		LimelightNetworkTable.getInstance().lightLED(true);
		LimelightNetworkTable.getInstance().setPipeline(pipeline);
		LimelightNetworkTable.getInstance().enable();
	}

	@Override
	public void onExecute() {
		double headingAdjustment = LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.DEGREE) 
				* Drive.kP_thetaOneD;
		
		double outputLeft, outputRight;
		
		outputLeft = -headingAdjustment;
		outputRight = headingAdjustment;
		
		out.pidWrite(outputLeft, outputRight);
	}
	
	
	@Override
	public void onEnd() {
		LimelightNetworkTable.getInstance().disable();
		LimelightNetworkTable.getInstance().lightLED(false);
	}

	@Override
	public boolean isFinishedNR() {
		return false;
	}
	
}
