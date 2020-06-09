package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * this class was made in 2020 by Nate Stringham and others to be used to the drawbridge and hang mechanisms on the 2020 robot
 * 
 * <p>this class should be passed a motor and the IDs of two DIO sensors that should be located at the ends of the motors range of travel
 * the class will try to keep the motor at one of the ends without burning out the motors
 * 
 * <p>this class is not meant to deal with motors that are connected to an encoder
 * 
 * <p>each object must have the tick() method called in robotPeriodic() to run properly
 */
class TwoStateMotor {
    double speed; // the speed that the motor should travel at
    double speedOffset = 0; // used to combat gravity, positive values cause faster sets and slower unsets
                           // (if speed is negative this value will have the opposite effect)
    TalonSRX motor; // the motor to be controlled
    DigitalInput defaultSensor; // the sensor that will be true when the motor is in the default position
    DigitalInput setSensor; // the sensor that will be true when the motor is in the set position
    boolean isDefault; // the state of the default Sensor
    boolean isSet; // the state of the set Sensor
    int direction = -1; // the target direction 1 for set and -1 for normal

    /**
     * @param speed the speed that the motor should travel at value between 1 and -1
     * @param motor the motor to be controlled
     * @param defaultSensor the sensor that will be true when the motor is in the default position
     * @param setSensor the sensor that will be true when the motor is in the set position
     */
    TwoStateMotor(double speed, TalonSRX motor, DigitalInput defaultSensor, DigitalInput setSensor) {
        this.speed = speed;
        this.motor = motor;
        this.defaultSensor = defaultSensor;
        this.setSensor = setSensor;
        this.motor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * @param speed the speed that the motor should travel at value between 1 and -1
     * @param speedOffset used to combat gravity, positive values cause faster sets and slower unsets 
     *                      (if speed is negative this value will have the opposite effect)
     * @param motor the motor to be controlled
     * @param defaultSensor the sensor that will be true when the motor is in the default position
     * @param setSensor the sensor that will be true when the motor is in the set position
     */
    TwoStateMotor(double speed, double speedOffset, TalonSRX motor, DigitalInput defaultSensor, DigitalInput setSensor) {
        this(speed, motor, defaultSensor, setSensor);
        this.speedOffset = speedOffset;
    }

    /**
     * this method should be called in robot periodic
     * checks sensor values and motor speed
     */
    public void tick() {
        
        isDefault = !defaultSensor.get();
        isSet = !setSensor.get();
        if (!isSet && direction == 1 || !isDefault && direction == -1) {
            motor.set(ControlMode.PercentOutput, (direction * speed) + speedOffset);
            //System.out.println("moving at "+ ((direction * speed) + speedOffset));
        } else {
            motor.set(ControlMode.PercentOutput, 0);
            //System.out.println("stopped");
        }
    }

    /**
     * call this function to change the limit that the motor will go to and stay at
     * 
     * @param set true if the motor should move toward the set position
     */
    public void set(boolean set) {
        if (set) {
            direction = 1;
        } else {
            direction = -1;
        }
    }
}   