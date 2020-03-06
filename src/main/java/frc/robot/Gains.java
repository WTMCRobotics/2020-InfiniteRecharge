/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot;

public class Gains {
	public double P;
	public double I;
	public double D;
	public final double F;
	public final int IZONE;
	public final double PEAK_OUTPUT;
	
	public Gains(double p, double i, double d, double f, int izone, double peakOutput){
		P = p;
		I = i;
		D = d;
		F = f;
		IZONE = izone;
		PEAK_OUTPUT = peakOutput;
	}
}
