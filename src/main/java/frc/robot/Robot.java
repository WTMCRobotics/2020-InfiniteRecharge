/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SolenoidBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.auton_instructions.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // ##########################################
    // auton related constants and variables
    // ##########################################

    // auton modes
    private static final int LEFT_AUTON_POS = 3;
    private static final int TARGET_ZONE_AUTON_POS = 2;
    private static final int TRENCH_AUTON_POS = 1;

    private static final int RIGHT_RENDEZVOUS = 1;
    private static final int LEFT_RENDEZVOUS = 2;
    private static final int TRENCH = 3;
    private static final int LOADING_ZONE = 4;

    // SendableChooser<String> puts a dropdown menu on the dashboard
    private final SendableChooser<Integer> STARTING_POS_CHOOSER = new SendableChooser<>();
    private int startingPosSelected; // the auton mode chossen by the dashboard
    // SendableChooser<String> puts a dropdown menu on the dashboard
    private final SendableChooser<Boolean> GO_DIRECTLY_CHOOSER = new SendableChooser<>();
    private boolean goDirectlyPosSelected; // the auton mode chossen by the dashboard
    // SendableChooser<String> puts a dropdown menu on the dashboard
    private final SendableChooser<Integer> TARGET_BALL_POS_CHOOSER = new SendableChooser<>();
    private int TargetBallPosSelected; // the auton mode chossen by the dashboard

    boolean shouldGoDirectlyToScore = true;

    int startingPosition = 0;
    static final int STARTING_POSITION_LEFT = 1;
    static final int STARTING_POSITION_CENTER = 2;
    static final int STARTING_POSITION_RIGHT = 3;
    int targetPickupLocation = 0;

    ArrayList<Instruction> autonInstructions = new ArrayList<Instruction>();

    // ##########################################
    // Digital IO related constants
    // ##########################################

    // DIO IDs
    static final int ROBOT_SENSOR_ID = 9; // this should be pulled low on the 2016 Practice Robot
    static final int HANG_SET_SENSOR_ID = 0; // sensor for when the winch is extended
    static final int HANG_DEFAULT_SENSOR_ID = 1; // sensor for when the winch is retracted
    static final int INTAKE_SENSOR_ID = 2; // sensor for when a ball is waitung to be popped up
    static final int INTAKE_COUNTER_SENSOR_ID = 3; // sensor for counting balls

    // Binary Sensors
    static final DigitalInput ROBOT_SENSOR = new DigitalInput(ROBOT_SENSOR_ID); // this should be pulled low on the 2016 Practice
                                                                  // Robot
    static final DigitalInput HANG_SET_SENSOR = new DigitalInput(HANG_SET_SENSOR_ID); // sensor for when the winch is
                                                                                      // extended
    static final DigitalInput HANG_DEFAULT_SENSOR = new DigitalInput(HANG_DEFAULT_SENSOR_ID); // sensor for when the
                                                                                              // winch is retracted
    static final DigitalInput INTAKE_SENSOR = new DigitalInput(INTAKE_SENSOR_ID); // sensor for when a ball is waitung
                                                                                  // to be popped up
    static final DigitalInput POPPER_SENSOR = new DigitalInput(INTAKE_COUNTER_SENSOR_ID); // sensor for counting balls

    // ##########################################
    // talon related constants and variables
    // ##########################################

    // can bus IDs. Can be found in Phoenix Tuner
    static final int LEFT_MASTER_ID = 2;
    static final int LEFT_SLAVE_ID = 3;
    static final int RIGHT_MASTER_ID = 4;
    static final int RIGHT_SLAVE_ID = 5;
    static final int WINCH_MOTOR_ID = 11;
    static final int INTAKE_ID = 12;
    static final int POPPER_ID = 10;

    // creates objects for the talons
    public TalonSRX leftMaster = new TalonSRX(LEFT_MASTER_ID);
    TalonSRX leftSlave = new TalonSRX(LEFT_SLAVE_ID);
    public TalonSRX rightMaster = new TalonSRX(RIGHT_MASTER_ID);
    TalonSRX rightSlave = new TalonSRX(RIGHT_SLAVE_ID);
    TalonSRX hangMotor = new TalonSRX(WINCH_MOTOR_ID);
    TalonSRX intake = new TalonSRX(INTAKE_ID);
    TalonSRX popper = new TalonSRX(POPPER_ID);

    static final int encoderRotation = 4096; // the number of ticks in a full rotation

    // talon config
    public static final int SLOT_IDX = 0; // Which PID slot to pull gains from
    public static final int PID_LOOP_IDX = 0; // Which PID loop to pull gains from
    public static final int TIMEOUT_MS = 30; // amount of time in ms to wait for conformation

    // ##########################################
    // drivetrain and pid related constants and variables
    // ##########################################

    // the obect that is the navX-MXP
    public AHRS gyro = new AHRS(Port.kMXP);
    static final Gains PRACTICE_ROTATION_GAINS = new Gains(0.004, 0.003, 0.001, 0.0, 0, 0.0);
    static final Gains COMPETITION_ROTATION_GAINS = new Gains(2.0, 0.0, 0.0, 0.0, 0, 0.0);
    static Gains rotationGains;
    static final Constraints ROTATIONAL_GAIN_CONSTRAINTS = new Constraints(Double.POSITIVE_INFINITY, 20); // m/s and
                                                                                                          // m/s/s
    ProfiledPIDController rotationPID;

    // The maximum distance from the destination considered close enough
    private static final double distanceMarginOfError = 0.5;

    // The margin of error for angles when turning in auton
    private static final double angleMarginOfError = 5;

    boolean isPracticeRobot; // true if ROBOT_SENSOR is pulled low
    double circumference; // this value will be updated with the circumference of the drive wheels

    boolean ArcadeDrive = true; // variable stores weather to use Arcade or tank style controls

    static final Gains PRACTICE_ROBOT_GAINS = new Gains(0.2, 0.00035, 1.5, 0.2, 0, 1.0);
    static final Gains COMPETITION_ROBOT_GAINS = new Gains(2.0, 0.0, 0.0, 0.2, 0, 1.0);
    static Gains gains; // used for drivetran motion magic when moving and is ste to
                        // PRACTICE_ROBOT_GAINS or COMPETITION_ROBOT_GAINS

    // ##########################################
    // intake and popper related constants and variables
    // ##########################################

    // the speed of the intake motor. Accepts values between 1 and -1.
    static final double INTAKE_SPEED_IN = 0.3;
    static final double INTAKE_SPEED_OUT = -0.25;

    // the speed of the popper motor. Accepts values between 1 and -1.
    static final double POPPER_SPEED_IN = 0.8;
    static final double POPPER_SPEED_OUT = -0.4;

    // the Amount of time popper motor should go for in robot cycles.
    static final int POPPER_TIME_IN = 50;
    static final int POPPER_TIME_OUT = 20;

    // the amount of time in robot cycles that this will move
    int popperInTime = 0;
    int popperOutTime = 0;

    int popperCounterTime; // the number of cycles that the counter sensor has bean interuped for
    static final int POPPER_COUNTER_COUNT_TIME = 5; // the number of cycles that a ball interupes the sensor for when
                                                    // passing
    static final int POPPER_COUNTER_JAM_TIME = 50; // the number of cycles that contitutes a popper jam
    int ballsStored = 0; // the number of balls in the robot

    // ##########################################
    // Drawbridge and hang related constants and variables
    // ##########################################

    // declares objects for the TwoStateMotor class
    TwoStateMotor hang = new TwoStateMotor(0.5, -0.1, hangMotor, HANG_DEFAULT_SENSOR, HANG_SET_SENSOR);;

    // ##########################################
    // Controller related constants and variables
    // ##########################################

    XboxController xboxController = new XboxController(0); // driver
    XboxController gHeroController = new XboxController(1); // co-driver
    double leftjoyY; // y-axis of the left joystick on the driver's controller
    double rightjoyY; // y-axis of the right joystick on the driver's controller
    double leftjoyX; // x-axis of the left joystick on the driver's controller
    double rightjoyX; // x-axis of the right joystick on the driver's controller

    static final int START = 7; // the mapping of the start button on a xbox controller
    static final int SELECT = 8; // the mapping of the select button on a xbox controller
    static final int R_STICK = 10; // the mapping of the right shoulder on a xbox controller
    static final int L_STICK = 9; // the mapping of the left shoulder on a xbox controller
    static final int R_SHOULDER = 6; // the mapping of the right shoulder on a xbox controller
    static final int L_SHOULDER = 5; // the mapping of the left shoulder on a xbox controller

    // these variables should be updated in teleopPeriodic()
    boolean arcadeButton; // true if the button that selects arcade mode is pressed
    boolean tankButton; // true if the button that selects tank mode is pressed
    boolean drawbridgeButton; // true if the button that lowers the drawbridge is pressed
    boolean hangButton; // true if the button that extends the hang mecanism is pressed
    boolean intakeInButton; // true if the button that intakes is pressed
    boolean intakeOutButton; // true if the button that runs the intake in reverse is pressed
    boolean popperInButton; // true if the button that runs the popper is pressed
    boolean popperOutButton; // true if the button that reverses the popper is pressed

    // ##########################################
    // Pneumatics related constants and variables
    // ##########################################

    static final int PCM_DRAWBRIDGE_IN = 1;
    static final int PCM_DRAWBRIDGE_OUT = 0;

    static final int PCM_RATCHET = 2;

    boolean extended = false;
    boolean retracted = true;

    Compressor compressor = new Compressor(1);

    DoubleSolenoid drawbridgeSol = new DoubleSolenoid(1, PCM_DRAWBRIDGE_IN, PCM_DRAWBRIDGE_OUT);
    Solenoid hangSol = new Solenoid(1, PCM_RATCHET);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        STARTING_POS_CHOOSER.addOption("Left (PS2)", LEFT_AUTON_POS);
        STARTING_POS_CHOOSER.addOption("Target Zone", TARGET_ZONE_AUTON_POS);
        STARTING_POS_CHOOSER.addOption("Trench", TRENCH_AUTON_POS);
        SmartDashboard.putData("Starting Position", STARTING_POS_CHOOSER);

        GO_DIRECTLY_CHOOSER.addOption("yes", true);
        GO_DIRECTLY_CHOOSER.addOption("no", false);
        SmartDashboard.putData("Go Directly to Target Zone", GO_DIRECTLY_CHOOSER);

        TARGET_BALL_POS_CHOOSER.addOption("Right Rendezvous", RIGHT_RENDEZVOUS);
        TARGET_BALL_POS_CHOOSER.addOption("Left Rendezvous", LEFT_RENDEZVOUS);
        TARGET_BALL_POS_CHOOSER.addOption("Trench", TRENCH);
        TARGET_BALL_POS_CHOOSER.addOption("Loading Zone", LOADING_ZONE);
        SmartDashboard.putData("Target Ball Position", TARGET_BALL_POS_CHOOSER);

        System.out.println("this is to test the drbug console and robotInit()");

        isPracticeRobot = !ROBOT_SENSOR.get();
        if (isPracticeRobot) {
            circumference = 6 * Math.PI;
            gains = PRACTICE_ROBOT_GAINS;
            rotationGains = PRACTICE_ROTATION_GAINS;
            System.out.println("using 6 inch weels");
        } else {
            circumference = 8 * Math.PI;
            gains = COMPETITION_ROBOT_GAINS;
            rotationGains = COMPETITION_ROTATION_GAINS;
            System.out.println("using 8 inch weels");
        }

        rotationPID = new ProfiledPIDController(rotationGains.P, rotationGains.I, rotationGains.D,
                ROTATIONAL_GAIN_CONSTRAINTS);

        initializeTalon(leftMaster, NeutralMode.Brake, false);
        initializeTalon(leftSlave, NeutralMode.Brake, false);
        initializeTalon(rightMaster, NeutralMode.Brake, true);
        initializeTalon(rightSlave, NeutralMode.Brake, true);
        initializeTalon(hangMotor, NeutralMode.Brake, false);
        initializeTalon(intake, NeutralMode.Coast, false);
        initializeTalon(popper, NeutralMode.Coast, false);

        initializeMotionMagicMaster(rightMaster);
        initializeMotionMagicMaster(leftMaster);

        rightSlave.set(ControlMode.Follower, RIGHT_MASTER_ID);
        leftSlave.set(ControlMode.Follower, LEFT_MASTER_ID);
        gyro.reset();
    }

    public void initializeTalon(TalonSRX talon, NeutralMode neutralMode, boolean inverted) {
        /* Ensure motor output is neutral during init */
        talon.set(ControlMode.PercentOutput, 0);

        /* Factory Default all hardware to prevent unexpected behaviour */
        talon.configFactoryDefault();

        /* Set Neutral mode */
        talon.setNeutralMode(neutralMode);

        /* Configure output direction */
        talon.setInverted(inverted);
    }

    public void initializeMotionMagicMaster(TalonSRX masterTalon) {
        /* Factory default hardware to prevent unexpected behavior */
        masterTalon.configFactoryDefault();

        /* Configure Sensor Source for Pirmary PID */
        masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_LOOP_IDX, TIMEOUT_MS);

        /*
         * set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %)
         */
        masterTalon.configNeutralDeadband(0.001, TIMEOUT_MS);

        /**
         * Configure Talon SRX Output and Sesnor direction accordingly Invert Motor to
         * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
         * sensor to have positive increment when driving Talon Forward (Green LED)
         */
        masterTalon.setSensorPhase(false);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT_MS);
        masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT_MS);

        /* Set the peak and nominal outputs */
        masterTalon.configNominalOutputForward(0, TIMEOUT_MS);
        masterTalon.configNominalOutputReverse(0, TIMEOUT_MS);
        masterTalon.configPeakOutputForward(1, TIMEOUT_MS);
        masterTalon.configPeakOutputReverse(-1, TIMEOUT_MS);

        /* Set Motion Magic gains in slot0 - see documentation */
        masterTalon.selectProfileSlot(SLOT_IDX, PID_LOOP_IDX);
        masterTalon.config_kF(SLOT_IDX, gains.F, TIMEOUT_MS);
        masterTalon.config_kP(SLOT_IDX, gains.P, TIMEOUT_MS);
        masterTalon.config_kI(SLOT_IDX, gains.I, TIMEOUT_MS);
        masterTalon.config_kD(SLOT_IDX, gains.D, TIMEOUT_MS);

        /* Set acceleration and vcruise velocity - see documentation */
        masterTalon.configMotionCruiseVelocity(15000, TIMEOUT_MS);
        masterTalon.configMotionAcceleration(6000, TIMEOUT_MS);

        /* Zero the sensor once on robot boot up */
        masterTalon.setSelectedSensorPosition(0, PID_LOOP_IDX, TIMEOUT_MS);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        hang.tick();

        // System.out.println("rightMaster.GetSelectedSensorPosition(): " +
        // rightMaster.getSelectedSensorPosition());
        // System.out.println("leftMaster.GetSelectedSensorPosition(): " +
        // leftMaster.getSelectedSensorPosition());

    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * STARTING_POSITION structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        autonInstructions.clear();
        startingPosSelected = STARTING_POS_CHOOSER.getSelected();
        System.out.println("starting Pos selected: " + startingPosSelected);
        goDirectlyPosSelected = GO_DIRECTLY_CHOOSER.getSelected();
        System.out.println("go directly selected: " + goDirectlyPosSelected);
        TargetBallPosSelected = TARGET_BALL_POS_CHOOSER.getSelected();
        System.out.println("target ball Pos selected: " + TargetBallPosSelected);
        resetEncoders();
        gyro.reset();
        rotationPID = new ProfiledPIDController(rotationGains.P, rotationGains.I, rotationGains.D,
                ROTATIONAL_GAIN_CONSTRAINTS);
        if (goDirectlyPosSelected) {
            if(startingPosSelected > 0){
                switch (startingPosSelected) {
                case TRENCH_AUTON_POS://start with front bumper on the initiation line facing our trench and start loaded with only two balls
                    autonInstructions.add(new MoveInch(133));
                    autonInstructions.add(new MoveInch(-133));
                    autonInstructions.add(new TurnDeg(90));
                    autonInstructions.add(new MoveInch(67.875));//67.875 inches might be the wrong distance from the middle of the trench run (the one closest to the target zone) to the middle of the target zone
                    autonInstructions.add(new TurnDeg(90));
                    autonInstructions.add(new MoveInch(120));
                case TARGET_ZONE_AUTON_POS:
                    //start with back bumper on initiation line facing torwards the middle of the target zone
                    autonInstructions.add(new MoveInch(-38));
                    autonInstructions.add(new MoveInch(120));
                case LEFT_AUTON_POS:
                    //start with back bumper on initiation line, middle of robot lined up with side of PS2 furthest from target zone facing away from end of arena closest to it
                    autonInstructions.add(new MoveInch(10));
                    autonInstructions.add(new TurnDeg(-90));
                    autonInstructions.add(new MoveInch(108));
                    autonInstructions.add(new TurnDeg(-90));
                    autonInstructions.add(new MoveInch(130));
                    break;
                default:
                    // this case shouldnt happen because of the if statment above
                    break;
                }
                autonInstructions.add(new StartPushing());
                autonInstructions.add(new SetPistonExtended(drawbridgeSol, true));
                autonInstructions.add(new WaitMs(3000));
                autonInstructions.add(new SetPistonExtended(drawbridgeSol, false));
                autonInstructions.add(new MoveInch(-120));
                switch (targetPickupLocation) {
                case RIGHT_RENDEZVOUS:
                    autonInstructions.add(new MoveInch(-308.625));
                    autonInstructions.add(new TurnDeg(-22.5));
                    autonInstructions.add(new MoveInch(60));
                    //picking up 2 balls here
                    autonInstructions.add(new MoveInch(-60));
                    autonInstructions.add(new TurnDeg(22.5));
                    autonInstructions.add(new MoveInch(308.625));
                    autonInstructions.add(new MoveInch(126));
                    //could put alternate code here making robot pick up other 3 balls
                    break;
                case LEFT_RENDEZVOUS:
                    autonInstructions.add(new TurnDeg(90));
                    //autonInstructions.add(new MoveInch(y));
                    autonInstructions.add(new TurnDeg(-90));
                    autonInstructions.add(new MoveInch(276));
                    autonInstructions.add(new TurnDeg(22.5));
                    autonInstructions.add(new MoveInch(60));
                    //picking up 3 balls here
                    autonInstructions.add(new MoveInch(-60));
                    autonInstructions.add(new TurnDeg(-22.5));
                    autonInstructions.add(new MoveInch(-270));
                    autonInstructions.add(new TurnDeg(-90));
                    //autonInstructions.add(new MoveInch(y));
                    autonInstructions.add(new TurnDeg(-90));
                    autonInstructions.add(new MoveInch(120));
                    //could put alternate code here making robot pick up other 2 balls
                    break;
                case TRENCH:
                    autonInstructions.add(new TurnDeg(-90));
                    autonInstructions.add(new MoveInch(67.875));
                    autonInstructions.add(new TurnDeg(90));
                    autonInstructions.add(new MoveInch(114));
                    //picking up 3 balls here
                    autonInstructions.add(new MoveInch(-114));
                    autonInstructions.add(new TurnDeg(90));
                    autonInstructions.add(new MoveInch(51.75));
                    autonInstructions.add(new TurnDeg(90));
                    autonInstructions.add(new MoveInch(126));                
                    break;
                case LOADING_ZONE:
                    //dont use this one
                    autonInstructions.add(new TurnDeg(180));
                    autonInstructions.add(new MoveInch(629.25));
                    autonInstructions.add(new StartPushing());
                    autonInstructions.add(new WaitMs(3000));
                    autonInstructions.add(new MoveInch(-629.25));
                    autonInstructions.add(new TurnDeg(180));
                    autonInstructions.add(new MoveInch(126));
                    break;
                default:
                    // TODO make fallback
                    break;
                }
            } else {
                // TODO make fallback
            }
        } else {
            //TODO
        }

    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        while (!autonInstructions.isEmpty() && autonInstructions.get(0).doit(this)) {
            autonInstructions.remove(0);
        }
        handelPopper(true);
        if (ballsStored < 5) {
            intake.set(ControlMode.PercentOutput, INTAKE_SPEED_IN);
        } else {
            intake.set(ControlMode.PercentOutput, INTAKE_SPEED_OUT);
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

        // this code updates the controller variables to the correct value at the
        // begining of teleopPeriodic()
        leftjoyY = xboxController.getY(GenericHID.Hand.kLeft);
        rightjoyY = xboxController.getY(GenericHID.Hand.kRight);
        leftjoyX = xboxController.getX(GenericHID.Hand.kLeft);
        rightjoyX = xboxController.getX(GenericHID.Hand.kRight);
        arcadeButton = xboxController.getRawButton(L_STICK);
        tankButton = xboxController.getRawButton(R_STICK);
        drawbridgeButton = 1 == gHeroController.getX(GenericHID.Hand.kRight);
        intakeInButton = 0.1 < xboxController.getTriggerAxis(GenericHID.Hand.kLeft);
        intakeOutButton = 0.1 < xboxController.getTriggerAxis(GenericHID.Hand.kRight);
        popperInButton = xboxController.getRawButton(L_SHOULDER);
        popperOutButton = xboxController.getRawButton(R_SHOULDER);
        hangButton = gHeroController.getName().isEmpty() ? false
                : 0.5 > gHeroController.getTriggerAxis(GenericHID.Hand.kLeft);

        setPistonExtended(drawbridgeSol, drawbridgeButton);

        hang.set(hangButton);
        setPistonExtended(hangSol, hangButton);

        if (arcadeButton) {
            ArcadeDrive = true;
        }
        if (tankButton) {
            ArcadeDrive = false;
        }

        // this code sets the motors to the correct speed based on driver input
        if (ArcadeDrive) {
            double x = rightjoyX;
            double y = leftjoyY;
            leftMaster.set(ControlMode.PercentOutput, (y * (2 - Math.abs(x)) - x * (2 - Math.abs(y))) / 2);
            rightMaster.set(ControlMode.PercentOutput, (y * (2 - Math.abs(x)) + x * (2 - Math.abs(y))) / 2);
        } else {
            leftMaster.set(ControlMode.PercentOutput, leftjoyY);
            rightMaster.set(ControlMode.PercentOutput, rightjoyY);
        }

        if (xboxController.getXButton()) {
            resetEncoders();
        }

        // this code handles intake
        if (intakeInButton && ballsStored < 5) {
            intake.set(ControlMode.PercentOutput, INTAKE_SPEED_IN);
        } else if (intakeOutButton || ballsStored >= 5) {
            intake.set(ControlMode.PercentOutput, INTAKE_SPEED_OUT);
        }
        // this code handles the popper
        if (popperInButton) {
            popper.set(ControlMode.PercentOutput, POPPER_SPEED_IN);
            handelPopper(false);
        } else if (popperOutButton) {
            popper.set(ControlMode.PercentOutput, POPPER_SPEED_OUT);
            handelPopper(false);
        } else {
            handelPopper(true);
        }

    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        setPistonExtended(drawbridgeSol, true);
    }

    public void testInit() {
    }

    // sets encoder position to zero
    public boolean resetEncoders() {
        ErrorCode rightError = rightMaster.setSelectedSensorPosition(0);
        ErrorCode leftError = leftMaster.setSelectedSensorPosition(0);
        return rightError.value == 0 && leftError.value == 0;
    }

    // move an amount in s straight line
    public boolean moveInches(double inches) {
        inches = -inches;
        leftMaster.set(ControlMode.MotionMagic, inchesToTicks(inches));
        rightMaster.set(ControlMode.MotionMagic, inchesToTicks(inches));
        if (Math.abs(leftMaster.getSelectedSensorPosition() - inchesToTicks(inches)) < inchesToTicks(
                distanceMarginOfError)
                && Math.abs(leftMaster.getActiveTrajectoryVelocity()) < inchesToTicks(1) * 10
                && Math.abs(rightMaster.getSelectedSensorPosition() - inchesToTicks(inches)) < inchesToTicks(
                        distanceMarginOfError)
                && Math.abs(rightMaster.getActiveTrajectoryVelocity()) < inchesToTicks(1) * 10) {
            return true;
        } else {
            return false;
        }
    }

    // converts inches to the value needed by the talon encoder for motion magic
    double inchesToTicks(double inches) {
        return encoderRotation * inches / circumference;
    }

    boolean turnRads(double radians) {
        return turnDegs(radians * 180 / Math.PI);
    }

    public boolean turnDegs(double degrees) {
        degrees %= 360;
        if (180 < degrees) {
            degrees -= 360;
        } else if (-180 > degrees) {
            degrees += 360;
        }
        double output = MathUtil.clamp(rotationPID.calculate(gyro.getAngle(), degrees), -1, 1);
        System.out.println(rotationPID.getPositionError() + "    " + output);
        if (output > 0) {
            output += 0.10;
        } else if (output < 0) {
            output -= 0.10;
        }
        if (Math.abs(gyro.getAngle() - degrees) < angleMarginOfError
                && Math.abs(rightMaster.getSelectedSensorVelocity()) < 1024 / 4
                && Math.abs(leftMaster.getSelectedSensorVelocity()) < 1024 / 4) {
            rightMaster.set(ControlMode.PercentOutput, 0);
            leftMaster.set(ControlMode.PercentOutput, 0);
            return true;
        } else {
            rightMaster.set(ControlMode.PercentOutput, output);
            leftMaster.set(ControlMode.PercentOutput, -output);
            return false;
        }
    }

    void UpdateCompressor() {
        // if not enough pressure
        if (!compressor.getPressureSwitchValue()) {
            // Start compressor
            compressor.start();
        }
        // if enough pressure
        else {
            // Stop compressor
            compressor.stop();
        }
    } // END of UpdateCompressor() functionom

    public void setPistonExtended(SolenoidBase solenoid, boolean value) {
        if (solenoid instanceof Solenoid) {
            ((Solenoid) solenoid).set(value);
        } else if (solenoid instanceof DoubleSolenoid) {
            if (value) {
                ((DoubleSolenoid) solenoid).set(DoubleSolenoid.Value.kForward);
            } else {
                ((DoubleSolenoid) solenoid).set(DoubleSolenoid.Value.kReverse);
            }

        }
    }

    // this code is called from auton and teleop periodic and uses sensors to
    // automaticly handel the popper
    void handelPopper(boolean shoudSetPopper) {
        // if a ball is ready to be popped
        if (!INTAKE_SENSOR.get()) {
            popperInTime = POPPER_TIME_IN;
        }

        // if there is a ball at the top of the popper
        if (!POPPER_SENSOR.get()) {
            if (++popperCounterTime > POPPER_COUNTER_JAM_TIME) {
                System.out.println("popper jammed");
                popperInTime = POPPER_TIME_IN;
                popperOutTime = POPPER_TIME_OUT;
            }
        } else {
            if (popperCounterTime > POPPER_COUNTER_JAM_TIME) {
                System.out.println("popper unjammed");
            } else if (popperCounterTime > POPPER_COUNTER_COUNT_TIME) {
                System.out.println("ball counted");
                ballsStored++;
                System.out.println("ballsStored: "+ballsStored);
            }
            popperCounterTime = 0;
        }

        if(shoudSetPopper){
            if (popperOutTime-- > 0) {
                popper.set(ControlMode.PercentOutput, POPPER_SPEED_OUT);
            } else if (popperInTime-- > 0 && ballsStored < 5) {
                popper.set(ControlMode.PercentOutput, POPPER_SPEED_IN);
            } else {
                popper.set(ControlMode.PercentOutput, 0);
            }
        }

        if (drawbridgeSol.get() == Value.kForward) {
            ballsStored = 0;
        }
    }
}