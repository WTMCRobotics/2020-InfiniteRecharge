package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

class TwoStateMotor {
    float speed; //the speed that the motor should travel at
    float speedOffset = 0; //used to combat gravity, positive values cause faster sets and slower unsets (if speed is negitive this value will have the opposite effect)
    TalonSRX motor; //the motor to be controlled
    DigitalInput defaultSensor; //the sensor that will be true when the motor is in the default position
    DigitalInput setSensor; //the sensor that will be true when the motor is in the set position
    boolean isDefault; //the state of the default Sensor
    boolean isSet; //the state of the set Sensor
    int direction; //the target direction 1 for set and -1 for normal

    TwoStateMotor (float speed, TalonSRX motor, int defaultSensor, int setSensor){
        this.speed = speed;
        this.motor = motor;
        this.defaultSensor = new DigitalInput(defaultSensor);
        this.setSensor = new DigitalInput(setSensor);
        this.motor.setNeutralMode(NeutralMode.Brake);
    }

    TwoStateMotor (float speed, float speedOffset, TalonSRX motor, int defaultSensor, int setSensor){
        this(speed, motor, defaultSensor, setSensor);
        this.speedOffset = speedOffset;
    }

    //this method should be called in robot periodic
    //checks sensor values and motor speed
    void tick(){

        isDefault =  defaultSensor.get();
        isSet =  setSensor.get();
        if(isSet && direction == 1 || isDefault && direction == -1) {
            motor.set(ControlMode.PercentOutput, (direction * speed) + speedOffset);
            System.out.println("moving at "+ ((direction * speed) + speedOffset));
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