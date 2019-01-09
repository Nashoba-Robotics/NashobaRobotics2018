package edu.nr.lib;


public abstract class Equation {
	
	public abstract double getValue(double x);
	
	public double defferentiate(double x) {
		double slope;
		
		slope = (getValue(x + 0.00000001) - getValue(x - 0.00000001)) / 0.00000002;
		slope = (Math.round(slope * 10000) / 10000);
		
		return slope;
		
	}
	
	public double integrate(double x1, double x2) {
		double totalArea = 0;
		double area;
		double value;
		
		for (int i = 0; x1 + ((i + 1) / 1000000.0) < x2; i++) {
			value = (getValue(x1 + (i / 1000000.0)) + getValue(x1 + ((i + 1) / 1000000.0))) / 2;
			
			area = value / 1000000.0;
			
			totalArea += area;
			
		}
		
		return (Math.round(totalArea * 10000.0) / 10000.0);
		
	}

}
