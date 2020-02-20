package frc.robot.autonInstructions;

import frc.robot.Robot;

public class TurnDeg extends Instruction {

	double degrees;

	public TurnDeg(double degrees) {
		this.degrees = degrees;
	}

	@Override
	public boolean doit(Robot robot) {
		if(robot.turnDegs(degrees) && robot.resetEncoders()){
			System.out.println("done");
			return true;
		}
		return false;
	}

}
