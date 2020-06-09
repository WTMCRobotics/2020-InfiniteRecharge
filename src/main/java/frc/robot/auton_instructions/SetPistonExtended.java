package frc.robot.auton_instructions;

import edu.wpi.first.wpilibj.SolenoidBase;
import frc.robot.Robot;

public class SetPistonExtended extends Instruction {

	boolean value;
	SolenoidBase solenoid;

	/**
     * @param solenoid the Solenoid or DoubleSolenoid to be extended or retracted
     * @param value whether the solenoid should be extended
     */
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
