package frc.robot.auton_instructions;

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
		if (startTime == -1) {
			System.out.println("waiting " + milliseconds / 1000 + " seconds");
			startTime = RobotController.getFPGATime() / 1000;
			return false;
		} else {
			return startTime + milliseconds <= RobotController.getFPGATime() / 1000;
		}
	}

}
