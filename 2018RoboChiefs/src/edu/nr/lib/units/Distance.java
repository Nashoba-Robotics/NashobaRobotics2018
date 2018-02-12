package edu.nr.lib.units;

import edu.nr.lib.Units;
import edu.nr.robotics.subsystems.climber.Climber;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandler;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;

public class Distance {
	
	public static final Distance ZERO = new Distance(0, Unit.defaultUnit);
	private double val;
	private Unit type;
	
	public enum Unit implements GenericUnit {
		FOOT, INCH, DRIVE_ROTATION, DRIVE_ROTATION_H, METER, 
		MAGNETIC_ENCODER_TICK, MAGNETIC_ENCODER_TICK_H, MAGNETIC_ENCODER_TICK_ELEV, MAGNETIC_ENCODER_TICK_INTAKE_ELEV,
		MAGNETIC_ENCODER_TICK_ELEV_SHOOTER, MAGNETIC_ENCODER_TICK_CUBE_HANDLER, MAGNETIC_ENCODER_TICK_INTAKE_ROLLERS,
		MAGNETIC_ENCODER_TICK_CLIMBER;
		
		public static final Unit defaultUnit = INCH;
		
		/**
		 * For a drive subsystem
		 */
		private static final double DRIVE_ROTATION_PER_INCH = 1/(Drive.EFFECTIVE_WHEEL_DIAMETER_INCHES * Math.PI);
		private static final double MAGNETIC_ENCODER_TICK_PER_INCH = Units.MAGNETIC_NATIVE_UNITS_PER_REV * DRIVE_ROTATION_PER_INCH / Units.NATIVE_UNITS_PER_TICK;
		
		/**
		 * For an h drive
		 */
		private static final double DRIVE_ROTATION_H_PER_INCH = 1/(Drive.EFFECTIVE_WHEEL_DIAMETER_INCHES_H * Math.PI);
		private static final double MAGNETIC_ENCODER_TICK_H_PER_INCH = Units.MAGNETIC_NATIVE_UNITS_PER_REV * DRIVE_ROTATION_H_PER_INCH / Units.NATIVE_UNITS_PER_TICK;
		
		/**
		 * For an elevator
		 */
		private static final double ENCODER_TICK_ELEV_PER_INCH = Elevator.ENC_TICK_PER_INCH_CARRIAGE;
		private static final double ENCODER_TICK_INTAKE_ELEV_PER_INCH = IntakeElevator.ENC_TICK_PER_INCH_INTAKE_ELEVATOR;
		
		/**
		 * For an elevator shooter
		 */
		private static final double ENCODER_TICK_ELEV_SHOOTER_PER_INCH = ElevatorShooter.ENC_TICK_PER_INCH_ELEVATOR_SHOOTER;
			
		/**
		 * For the intake rollers
		 */
		private static final double ENCODER_TICK_INTAKE_ROLLERS_PER_INCH = IntakeRollers.ENC_TICK_PER_INCH_INTAKE_ROLLERS;
		
		/**
		 * For the climber
		 */
		private static final double ENCODER_TICK_CLIMBER_PER_INCH = Climber.ENC_TICKS_PER_INCH_CLIMBER;
		
		private static final double FOOT_PER_INCH = 1.0/Units.INCHES_PER_FOOT;
		private static final double METER_PER_INCH = 1.0/Units.INCHES_PER_METER;
		
		public double convertToDefault(double val) {
			if(this == Unit.defaultUnit) {
				return val;
			}
			if(this == Unit.FOOT) {
				return val / FOOT_PER_INCH;
			}
			if(this == Unit.DRIVE_ROTATION) {
				return val / DRIVE_ROTATION_PER_INCH;
			}
			if(this == Unit.DRIVE_ROTATION_H) {
				return val / DRIVE_ROTATION_H_PER_INCH;
			}
			if(this == Unit.METER) {
				return val / METER_PER_INCH;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK) {
				return val / MAGNETIC_ENCODER_TICK_PER_INCH;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_H) {
				return val / MAGNETIC_ENCODER_TICK_H_PER_INCH;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_ELEV) {
				return val / ENCODER_TICK_ELEV_PER_INCH;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV) {
				return val / ENCODER_TICK_INTAKE_ELEV_PER_INCH;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_ELEV_SHOOTER) {
				return val / ENCODER_TICK_ELEV_SHOOTER_PER_INCH;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_INTAKE_ROLLERS) {
				return val / ENCODER_TICK_INTAKE_ROLLERS_PER_INCH;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_CLIMBER) {
				return val / ENCODER_TICK_CLIMBER_PER_INCH;
			}
			return 0;
		}
		
		public double convertFromDefault(double val) {
			if(this == Unit.defaultUnit) {
				return val;
			}
			if(this == Unit.FOOT) {
				return FOOT_PER_INCH * val;
			}
			if(this == Unit.DRIVE_ROTATION) {
				return DRIVE_ROTATION_PER_INCH * val;
			}
			if (this == Unit.DRIVE_ROTATION_H) {
				return DRIVE_ROTATION_H_PER_INCH * val;
			}
			if(this == Unit.METER) {
				return METER_PER_INCH * val;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK) {
				return MAGNETIC_ENCODER_TICK_PER_INCH * val;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_H) {
				return MAGNETIC_ENCODER_TICK_H_PER_INCH * val;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_ELEV) {
				return ENCODER_TICK_ELEV_PER_INCH * val;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV) {
				return ENCODER_TICK_INTAKE_ELEV_PER_INCH * val;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_ELEV_SHOOTER) {
				return ENCODER_TICK_ELEV_SHOOTER_PER_INCH * val;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_INTAKE_ROLLERS) {
				return ENCODER_TICK_INTAKE_ROLLERS_PER_INCH * val;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_CLIMBER) {
				return ENCODER_TICK_CLIMBER_PER_INCH * val;
			}
			return 0;
		}
}
	
	public Distance(double val, Unit type) {
		this.val = val;
		this.type = type;
	}
	
	public double get(Unit toType) {
		return type.convert(val, toType);
	}

	public double getDefault() {
		return get(Unit.defaultUnit);
	}
	
	public Distance sub(Distance distanceTwo) {
		return new Distance(this.get(Unit.defaultUnit) - distanceTwo.get(Unit.defaultUnit), Unit.defaultUnit);
	}
	
	public Distance add(Distance distanceTwo) {
		return new Distance(this.get(Unit.defaultUnit) + distanceTwo.get(Unit.defaultUnit), Unit.defaultUnit);
	}
	
	public Distance mul(double x) {
		return new Distance(this.get(Unit.defaultUnit) * x, Unit.defaultUnit);
	}
	
	public boolean lessThan(Distance distanceTwo) {
		return this.get(Unit.defaultUnit) < distanceTwo.get(Unit.defaultUnit);
	}

	public boolean greaterThan(Distance distanceTwo) {
		return this.get(Unit.defaultUnit) > distanceTwo.get(Unit.defaultUnit);
	}
	
	public Distance negate() {
		return new Distance(-this.get(Unit.defaultUnit), Unit.defaultUnit);
	}
	
	public Distance abs() {
		return new Distance(Math.abs(this.get(Unit.defaultUnit)), Unit.defaultUnit);
	}
	
	public double signum() {
		return Math.signum(this.get(Unit.defaultUnit));
	}
	
	@Override
	public boolean equals(Object distanceTwo) {
		if(distanceTwo instanceof Distance) {
			return this.get(Unit.defaultUnit) == ((Distance) distanceTwo).get(Unit.defaultUnit);
		} else {
			return false;
		}
	}

	public double div(Distance distance) {
		return this.get(Unit.defaultUnit) / distance.get(Unit.defaultUnit);
	}

}
