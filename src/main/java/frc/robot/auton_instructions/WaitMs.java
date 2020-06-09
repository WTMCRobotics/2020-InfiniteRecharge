package frc.robot.auton_instructions;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;

/**
 * wait an amount of time before continuing auton instructions
 */
public class WaitMs extends Instruction {

	long milliseconds;

	/**
	 * the time when doit was first called in milliseconds or -1 if doit has not bean called
	 */
	long startTime = -1;

	/**
	 * @param milliseconds the number of milliseconds to wait for (1 second = 1000 milliseconds)
	 */
	public WaitMs(long milliseconds) {
		this.milliseconds = milliseconds;
	}

	@Override
	public boolean doit(Robot robot) {
		if (startTime == -1) {
			System.out.println("waiting " + milliseconds / 1000 + " seconds");
			startTime = RobotController.getFPGATime() / 1000;
			return false;
		} else {
			return startTime + milliseconds <= RobotController.getFPGATime() / 1000;
		}
	}

}
