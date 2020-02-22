package frc.robot.auton_instructions;

import frc.robot.Robot;

public class MoveInch extends Instruction {

	double inches;

	public MoveInch(double inches) {
		this.inches = inches;
	}

	@Override
	public boolean doit(Robot robot) {
		return robot.moveInches(inches) && robot.resetEncoders();
	}

}
