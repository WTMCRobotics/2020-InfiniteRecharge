package frc.robot.auton_instructions;

import frc.robot.Robot;

public class TurnDeg extends Instruction {

	double degrees;

	/**
	 * @param degrees the number of degres to turn
	 */
	public TurnDeg(double degrees) {
		this.degrees = degrees;
	}

	@Override
	public boolean doit(Robot robot) {
		if(robot.turnDegs(degrees) && robot.resetEncoders()){
			robot.gyro.reset();
			System.out.println("done");
			return true;
		}
		return false;
	}

}
