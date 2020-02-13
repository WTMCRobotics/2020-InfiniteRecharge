package frc.robot.autonInstructions;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;

public class WaitMs extends Instruction {

	long milliseconds;

	long startTime = -1;

	public WaitMs(long milliseconds) {
		this.milliseconds = milliseconds;
	}

	@Override
	public boolean doit(Robot robot) {
		System.out.println("waiting " + milliseconds / 1000 + " seconds");
		if (startTime == -1) {
			startTime = RobotController.getFPGATime();
			return false;
		} else {
			return startTime + milliseconds <= RobotController.getFPGATime();
		}
	}

}
