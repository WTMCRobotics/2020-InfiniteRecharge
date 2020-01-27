/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot;

public class Gains {
	public final double P;
	public final double I;
	public final double D;
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
