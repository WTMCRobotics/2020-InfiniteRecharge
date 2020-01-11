package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

class TwoStateMotor {
    float speed;
    TalonSRX motor;
    DigitalInput defaultSensor, setSensor;
    boolean isDefault, isSet;
    int direction;

    TwoStateMotor (float speed, TalonSRX motor, int defaultSensor, int setSensor){
        this.speed = speed;
        this.motor = motor;
        this.defaultSensor = new DigitalInput(defaultSensor);
        this.setSensor = new DigitalInput(setSensor);
        this.motor.setNeutralMode(NeutralMode.Brake);
    }

    //this method should be called in robot periodic
    //checks sensor values and motor speed
    void tick(){

        isDefault =  defaultSensor.get();
        isSet =  setSensor.get();
        if(isSet && direction == 1 || isDefault && direction == -1) {
            motor.set(ControlMode.PercentOutput, direction * speed);
            System.out.println("moving at "+direction * speed);
        } else {
            motor.set(ControlMode.PercentOutput, 0);
            System.out.println("stopped");
        }
    }

    void set (boolean set) {
        if(set) {
            direction = 1;
        }else{
            direction = -1;
        }
    }
}