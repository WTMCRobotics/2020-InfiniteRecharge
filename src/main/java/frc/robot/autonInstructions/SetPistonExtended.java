package frc.robot.autonInstructions;

import edu.wpi.first.wpilibj.SolenoidBase;
import frc.robot.Robot;

public class SetPistonExtended extends Instruction {

	boolean value;
	SolenoidBase solenoid;

	public SetPistonExtended(SolenoidBase solenoid, boolean value) {
		this.solenoid = solenoid;
		this.value = value;
	}

	@Override
	public boolean doit(Robot robot) {
		System.out.println("extending: " + solenoid);
		robot.setPistonExtended(solenoid, value);
		return true;
	}

}
