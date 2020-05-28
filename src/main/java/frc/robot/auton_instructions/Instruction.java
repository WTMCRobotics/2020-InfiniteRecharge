package frc.robot.auton_instructions;

import frc.robot.Robot;

public abstract class Instruction {

	/**
	 * does the instruction
	 * 
	 * @param robot the current instance of the robot
	 * 
	 * @return whether the instruction has bean completed
	 */
	public abstract boolean doit(Robot robot);

}
