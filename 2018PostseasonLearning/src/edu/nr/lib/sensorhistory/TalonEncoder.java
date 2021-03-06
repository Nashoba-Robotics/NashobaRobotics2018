package edu.nr.lib.sensorhistory;

import java.util.LinkedList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.AngularSpeed;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;

public class TalonEncoder extends TimerTask {

	private final Timer timer;

	private final int PID_TYPE = 0; //0 = primary, 1 = cascade
	// In milliseconds
	private final Time period;
	private static final Time defaultPeriod = new Time(5, Time.Unit.MILLISECOND); // 200
																					// Hz

	int maxNumPts = 200;
	
	Distance.Unit encoderUnit;
	
	TalonSRX talon;

	List<Data> data;

	public TalonEncoder(TalonSRX talon, Distance.Unit encoderUnit) {
		this.talon = talon;
		this.encoderUnit = encoderUnit;
		
		this.period = defaultPeriod;

		data = new LinkedList<>();

		timer = new Timer();
		timer.schedule(this, 0, (long) this.period.get(Time.Unit.MILLISECOND));

	}

	@Override
	public void run() {
		if(data.size() > maxNumPts) {
			data.remove(0);
		}
		data.add(new Data(new Distance(talon.getSelectedSensorPosition(PID_TYPE), encoderUnit),
				new Speed(talon.getSelectedSensorVelocity(PID_TYPE), encoderUnit, Time.Unit.HUNDRED_MILLISECOND), Time.getCurrentTime()));
	}

	/**
	 * Get the position at a time in the past.
	 * 
	 * @param deltaTime
	 *            How long ago to look, in milliseconds
	 * @return the position in encoder ticks
	 */
	public Distance getPosition(Time deltaTime) {

		if (deltaTime.equals(Time.ZERO)) {
			return new Distance(talon.getSelectedSensorPosition(PID_TYPE), encoderUnit);
		}

		if (data.size() == 0) {
			return new Distance(talon.getSelectedSensorPosition(PID_TYPE), encoderUnit);
		} else if (data.size() == 1) {
			return data.get(0).position;
		}

		Time timestamp = Time.getCurrentTime().sub(deltaTime);

		int low = 0;
		int up = data.size() - 1;
		while (low < up)
		// @loop_invariant 0 <= low && low <= up && up <= n;
		// @loop_invariant low == 0 || A[low-1] < x;
		// @loop_invariant up == n || A[up] >= x;
		{
			// int mid = (low + up)/2; CAUSES OVERFLOW
			int mid = low + (up - low) / 2;
			if (timestamp.equals(data.get(mid).timestamp))
				return data.get(mid).position;
			else if (timestamp.lessThan(data.get(mid).timestamp))
				up = mid;
			else
				low = mid + 1;
		}
		low = up - 1; // This is so that low != up

		if (low == -1) {
			// We haven't been running for long enough.
			return data.get(up).position;
		}

		Data first = data.get(low);
		Data second = data.get(up);
		if (first.timestamp.equals(second.timestamp)) {
			System.out.println("The timestamps are equal in " + this + ". This is weird and unexpected...");
			return Distance.ZERO;
		}
		return new Distance(interpolate(first.position.get(encoderUnit), second.position.get(encoderUnit), timestamp.div(second.timestamp.add(first.timestamp))), encoderUnit);

	}

	/**
	 * Get the velocity at a time in the past.
	 * 
	 * @param deltaTime
	 *            How long ago to look
	 * @return the velocity
	 */
	public Speed getVelocity(Time deltaTime) {
	
		if (deltaTime.equals(Time.ZERO)) {
			return new Speed(talon.getSelectedSensorVelocity(PID_TYPE), encoderUnit, Time.Unit.HUNDRED_MILLISECOND);
		}
	
		if (data.size() == 0) {
			return new Speed(talon.getSelectedSensorVelocity(PID_TYPE), encoderUnit, Time.Unit.HUNDRED_MILLISECOND);
		} else if (data.size() == 1) {
			return data.get(0).velocity;
		}
	
		Time timestamp = Time.getCurrentTime().sub(deltaTime);
	
		int low = 0;
		int up = data.size() - 1;
		while (low < up)
		// @loop_invariant 0 <= low && low <= up && up <= n;
		// @loop_invariant low == 0 || A[low-1] < x;
		// @loop_invariant up == n || A[up] >= x;
		{
			// int mid = (low + up)/2; CAUSES OVERFLOW
			int mid = low + (up - low) / 2;
			if (timestamp.equals(data.get(mid).timestamp))
				return data.get(mid).velocity;
			else if (timestamp.lessThan(data.get(mid).timestamp))
				up = mid;
			else
				low = mid + 1;
		}
		low = up - 1; // This is so that low != up

		if (low == -1) {
			// We haven't been running for long enough.
			return data.get(up).velocity;
		}

		Data first = data.get(low);
		Data second = data.get(up);
		if (first.timestamp.equals(second.timestamp)) {
			System.out.println("The timestamps are equal in " + this + ". This is weird and unexpected...");
			return Speed.ZERO;
		}
		return new Speed(
				interpolate(first.velocity.getDefault(), second.velocity.getDefault(),
						timestamp.div(second.timestamp.add(first.timestamp))),
				Distance.Unit.defaultUnit, Time.Unit.defaultUnit);
	}

	private double interpolate(double first, double second, double timeRatio) {
		return first * (1 - timeRatio) + second * timeRatio;
	}

	private class Data {
		Distance position;
		Speed velocity;

		Time timestamp;

		public Data(Distance position, Speed velocity, Time timestamp) {
			this.position = position;
			this.velocity = velocity;
			this.timestamp = timestamp;
		}
	}

}
