import java.util.ArrayList;

public class Main {

	public static ArrayList<Double> posPoints;
	public static ArrayList<Double> velPoints;
	public static ArrayList<Double> accelPoints;
	
	public static double positionGoal;
	public static double velocityGoal;
	public static double accelGoal;
	
	private static RampedDiagonalHTrajectory trajectory;
	
	public static double period = 20; //ms
	
	public static double goalX = 10;
	public static double goalY = 0;
	public static double velMax = 10;
	public static double accelMax = 20;
	
	public static void main(String[] args) {
		
		trajectory = new RampedDiagonalHTrajectory(goalX, goalY, velMax, accelMax);
		
		posPoints = new ArrayList<Double>();
		velPoints = new ArrayList<Double>();
		accelPoints = new ArrayList<Double>();
		
		posPoints = trajectory.loadPosPoints(period);
		velPoints = trajectory.loadVelPoints(period);
		accelPoints = trajectory.loadAccelPoints(period);
		
		for (int loopIteration = 0; loopIteration < posPoints.size(); loopIteration++) {
			positionGoal = posPoints.get(loopIteration);
			System.out.println(positionGoal);
		}
		
		for (int loopIteration = 0; loopIteration < posPoints.size(); loopIteration++) {
			velocityGoal = velPoints.get(loopIteration);
			System.out.println(velocityGoal);
		}
		
		for (int loopIteration = 0; loopIteration < posPoints.size(); loopIteration++) {
			accelGoal = accelPoints.get(loopIteration);
			System.out.println(accelGoal);
		}
		
		
	}

}
