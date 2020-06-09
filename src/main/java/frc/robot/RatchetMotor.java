package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

public class RatchetMotor extends TwoStateMotor {

    int backwardTime = 0;

    RatchetMotor(double speed, TalonSRX motor, DigitalInput defaultSensor, DigitalInput setSensor) {
        super(speed, motor, defaultSensor, setSensor);
    }
    RatchetMotor(double speed, double speedOffset, TalonSRX motor, DigitalInput defaultSensor, DigitalInput setSensor) {
        super(speed, speedOffset, motor, defaultSensor, setSensor);
    }

    // this method should be called in robot periodic
    // checks sensor values and motor speed
    public void tick() {
        
        isDefault = defaultSensor.get();
        isSet = setSensor.get();
        if(--backwardTime > 0){
            motor.set(ControlMode.PercentOutput, ((direction * speed ) + speedOffset) * -1);
        } else if (isSet && direction == 1 || isDefault && direction == -1) {
            motor.set(ControlMode.PercentOutput, (direction * speed) + speedOffset);
            //System.out.println("moving at "+ ((direction * speed) + speedOffset));
        } else {
            motor.set(ControlMode.PercentOutput, 0);
            //System.out.println("stopped");
        }
    }

    // call this function to change the limit that the motor will go to and stay at
    public void set(boolean set) {
        if (set && direction != 1) {
            backwardTime = 25;
            direction = 1;
        } else if (direction != -1) {
            backwardTime = 25;
            direction = -1;
        }
    }
}