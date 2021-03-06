package edu.nr.lib.network;

import java.util.Timer;
import java.util.TimerTask;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Time;
import edu.nr.lib.units.Angle.Unit;
import edu.wpi.first.networktables.*;

/**
 * Class to get limelight values. Enable and disable to keep code running quickly
 * 
 * @author Nashoba1768 (Ben)
 *
 */
public class LimelightNetworkTable extends TimerTask {
	
	private static final Time DEFAULT_PERIOD = new Time(10, Time.Unit.MILLISECOND);
	private static final boolean DEFAULT_LED_LIGHT = false;
	private static final Pipeline DEFAULT_PIPELINE = Pipeline.PowerCube;
	
	private static final Time IMAGE_CAPTURE_LATENCY = new Time(11, Time.Unit.MILLISECOND);
	
	private double boxHeight = 0;
	private Angle horizOffsetAngle = Angle.ZERO;
	private Angle vertOffsetAngle = Angle.ZERO;
	private Time pipelineLatency = Time.ZERO;
	
	private boolean enabled = false;
	
	private final Timer timer;
	
	private NetworkTable limelightTable;
	
	private static LimelightNetworkTable singleton;
	
	public enum Pipeline {
		PowerCube
	}
	
	public static LimelightNetworkTable getInstance() {
		if(singleton == null)
			init();
		return singleton;
	}
	
	public synchronized static void init() {
		if(singleton == null) {
			singleton = new LimelightNetworkTable();
		}
	}
	
	private LimelightNetworkTable() {
		timer = new Timer();
		timer.schedule(this, 0, (long) DEFAULT_PERIOD.get(Time.Unit.MILLISECOND));
		limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
		lightLED(DEFAULT_LED_LIGHT);
		setPipeline(DEFAULT_PIPELINE);
	}
	
	@Override
	public void run() {
			boxHeight = limelightTable.getEntry("tvert").getDouble(0);
			vertOffsetAngle = new Angle(limelightTable.getEntry("ty").getDouble(0), Angle.Unit.DEGREE);
			horizOffsetAngle = new Angle(limelightTable.getEntry("tx").getDouble(0), Angle.Unit.DEGREE);
			pipelineLatency = new Time(limelightTable.getEntry("tl").getDouble(0), Time.Unit.MILLISECOND);
			//System.out.println("vert: " + vertOffsetAngle.get(Angle.Unit.DEGREE) + '\n' + "horiz: " + horizOffsetAngle.get(Unit.DEGREE) + '\n' + "latency: " + pipelineLatency.get(Time.Unit.SECOND));
	}
	
	/**
	 * Enables returning from limelight
	 */
	public void enable() {
		enabled = true;
	}
	
	/**
	 * Disables values returning from limelight
	 */
	public void disable() {
		enabled = false;
	}

	/**
	 * 
	 * @return Limelight target box height
	 */
	public double getBoxHeight() {
		return boxHeight;
	}
	
	/**
	 * 
	 * @return Limelight horizontal offset angle or 0
	 */
	public Angle getHorizOffset() {
		return horizOffsetAngle;
	}
	
	/**
	 * 
	 * @return Limelight horizontal offset angle or Double.MAX_VALUE
	 */
	public Angle getVertOffsetAngle() {
		return vertOffsetAngle;
	}
	
	/**
	 * 
	 * @return Limelight image latency
	 */
	public Time getImageLatency() {
		return pipelineLatency.add(IMAGE_CAPTURE_LATENCY);
	}

	public void lightLED(boolean bool) {
		if (bool) {
			limelightTable.getEntry("ledMode").setDouble(3);
		} else {
			limelightTable.getEntry("ledMode").setDouble(1);
		}
	}
	
	public void setPipeline(Pipeline pipeline) {
		if (pipeline == Pipeline.PowerCube) {
			limelightTable.getEntry("pipeline").setDouble(0);
		}
	}
	
	public double getLED() {
		return limelightTable.getEntry("ledMode").getDouble(-1);
	}
}
