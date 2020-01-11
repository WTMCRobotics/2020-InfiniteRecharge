/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    static final int LEFT_MASTER_ID = 2;
	static final int LEFT_SLAVE_ID = 3;

	static final int RIGHT_MASTER_ID = 4;
    static final int RIGHT_SLAVE_ID = 5;
    
    static final int DRAWBRIDGE_MOTOR_ID = 6;
    static final int WINCH_MOTOR_ID = 7;

    static final int DRAWBRIDGE_SET_SENSOR = 3;
    static final int DRAWBRIDGE_DEFAULT_SENSOR = 4;
    static final int HANG_SET_SENSOR = 1;
    static final int HANG_DEFAULT_SENSOR = 2;
    
    XboxController xboxController = new XboxController(0);
    double leftjoyY;
    double rightjoyY;
    double leftjoyX;
    double rightjoyX;
    boolean ArcadeDrive = true;

    static final int START = 7;
    static final int SELECT = 8;
    static final int R_SHOULDER = 6;
    static final int L_SHOULDER = 5;
    boolean start;
    boolean select;
    boolean drawbridgeButton;
    boolean hangButton;


    TalonSRX leftMaster = new TalonSRX(LEFT_MASTER_ID);
	TalonSRX leftSlave = new TalonSRX(LEFT_SLAVE_ID);
	TalonSRX rightMaster = new TalonSRX(RIGHT_MASTER_ID);
    TalonSRX rightSlave = new TalonSRX(RIGHT_SLAVE_ID);
    TalonSRX drawbridgeMotor = new TalonSRX(DRAWBRIDGE_MOTOR_ID);
    TalonSRX hangMotor = new TalonSRX(WINCH_MOTOR_ID);
    
    TwoStateMotor drawbridge;
    TwoStateMotor hang;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
        System.out.println("this is to test the drbug console and robotInit()");

        drawbridge = new TwoStateMotor(-1, drawbridgeMotor, DRAWBRIDGE_DEFAULT_SENSOR, DRAWBRIDGE_SET_SENSOR);
        hang = new TwoStateMotor(-1, hangMotor, HANG_DEFAULT_SENSOR, HANG_SET_SENSOR);

        /* Ensure motor output is neutral during init */
		leftMaster.set(ControlMode.PercentOutput, 0);
		rightMaster.set(ControlMode.PercentOutput, 0);        

		/* Factory Default all hardware to prevent unexpected behaviour */
		leftMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();
		rightMaster.configFactoryDefault();
		rightSlave.configFactoryDefault();
		
		/* Set Neutral mode */
		leftMaster.setNeutralMode(NeutralMode.Brake);
		leftSlave.setNeutralMode(NeutralMode.Brake);
		rightMaster.setNeutralMode(NeutralMode.Brake);
		rightSlave.setNeutralMode(NeutralMode.Brake);
		
		/* Configure output direction */
		leftMaster.setInverted(false);
        leftSlave.setInverted(false);
		rightMaster.setInverted(false);
        leftSlave.setInverted(false);

        rightSlave.set(ControlMode.Follower, RIGHT_MASTER_ID);
		leftSlave.set(ControlMode.Follower, LEFT_MASTER_ID);
    }
    
    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        drawbridge.set(drawbridgeButton);
        drawbridge.tick();

        hang.set(hangButton);
        hang.tick();
    }
    
    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
    }
    
    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case kCustomAuto:
            // Put custom auto code here
            break;
            case kDefaultAuto:
            default:
            // Put default auto code here
            break;
        }
    }
    
    /**
     * This function is called periodically during teleop.
     */
    @Override
    public void teleopInit() {
    }


    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        leftjoyY = xboxController.getY(GenericHID.Hand.kLeft);
        rightjoyY = xboxController.getY(GenericHID.Hand.kRight);
        leftjoyX = xboxController.getX(GenericHID.Hand.kLeft);
        rightjoyX = xboxController.getX(GenericHID.Hand.kRight);
        start = xboxController.getRawButton(START);
        select = xboxController.getRawButton(SELECT);
        drawbridgeButton = xboxController.getRawButton(R_SHOULDER);
        hangButton = xboxController.getRawButton(L_SHOULDER);
        if (start) {
            ArcadeDrive = true;
        }
        if (select) {
            ArcadeDrive = false;
        }

        if (ArcadeDrive) {
            leftMaster.set(ControlMode.PercentOutput, -(leftjoyY - leftjoyX)/2);
		    rightMaster.set(ControlMode.PercentOutput, (leftjoyY + leftjoyX)/2);
        } else {
            leftMaster.set(ControlMode.PercentOutput, -leftjoyY);
		    rightMaster.set(ControlMode.PercentOutput, rightjoyY);
        }
    }
    
    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
