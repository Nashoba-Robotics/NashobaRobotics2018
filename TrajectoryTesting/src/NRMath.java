

public class NRMath {

	/**
	 * Squares the input value and maintains the sign
	 * 
	 * @param x
	 * @return the squared value with the sign maintained
	 */
	public static double squareWithSign(double x) {
		return powWithSign(x, 2);
	}

	/**
	 * Returns the value of the first argument raised to the power of the second
	 * argument multiplied by the sign of the first argument
	 * 
	 * @param x
	 *            the base
	 * @param exp
	 *            the exponent
	 * @return the value a^b * the sign of a
	 */
	public static double powWithSign(double x, double exp) {
		return Math.abs(Math.pow(x, exp)) * Math.signum(x);
	}

	/**
	 * Limits x from -y to y
	 * 
	 * @param x
	 * @param y
	 * @return the value, limited from -y to y
	 */
	public static double limit(double x, double y) {
		if (x > y) {
			return y;
		}
		if (x < -y) {
			return -y;
		}
		return x;
	}

	/**
	 * Limits x from -1 to 1
	 * 
	 * @param x
	 * @return the value, limited between -1 and 1
	 */
	public static double limit(double x) {
		return limit(x, 1);
	}
	
	public static double round(double value, double decimals) {
		return Math.round(value * Math.pow(10.0, decimals)) / Math.pow(10.0, decimals);
	}

	public static double quadratic(double a, double b, double c, boolean higher) {
		double quadPos = (-b + Math.sqrt(Math.pow(b, 2) - 4 * a * c)) / (2 * a);
		double quadNeg = (-b - Math.sqrt(Math.pow(b, 2) - 4 * a * c)) / (2 * a);
		if (higher) {
			return quadPos;
		}
		return quadNeg;
	}
}
