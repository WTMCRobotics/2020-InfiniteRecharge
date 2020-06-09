package frc.robot.auton_instructions;

import frc.robot.Robot;

public class MoveInch extends Instruction {

	double inches;

	/**
	 * @param inches the number of inches to move forward (negitve value to go backward)
	 */
	public MoveInch(double inches) {
		this.inches = inches;
	}

	@Override
	public boolean doit(Robot robot) {
		return robot.moveInches(inches) && robot.resetEncoders();
	}

}
