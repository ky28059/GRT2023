package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.motorcontrol.MotorUtil;

import static frc.robot.Constants.TiltedElevatorConstants.*;

public class TiltedElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax extensionMotor;
    private final RelativeEncoder extensionEncoder;
    private final SparkMaxPIDController extensionPidController;

    private final CANSparkMax extensionFollow;

    private final DigitalInput zeroLimitSwitch = new DigitalInput(ZERO_LIMIT_ID);

    private static final double EXTENSION_GEAR_RATIO = 14.0 / 64.0;
    private static final double EXTENSION_CIRCUMFERENCE = Units.inchesToMeters(Math.PI * 0.500); // approx circumference of winch
    private static final double EXTENSION_ROTATIONS_TO_METERS = EXTENSION_GEAR_RATIO * EXTENSION_CIRCUMFERENCE * 2.0 * (15.0 / 13.4);
    private static final float EXTENSION_LIMIT = 26.0f;

    private double extensionP = 0.25; //.4 // 4.9;
    private double extensionI = 0;
    private double extensionD = 0.9; //.2
    private double extensionFF = 0.3; //0.1
    private double extensionTolerance = 0.003;
    private double arbFeedforward = 0.02;

    private double maxVel = 1.1; // m/s
    private double maxAccel = 1.8; // 0.6 // m/s^2
    private double targetExtension = 0;

    private ElevatorState currentState = ElevatorState.GROUND;
    private double manualPower = 0;
    private double offsetDistMeters = 0;

    private static final double OFFSET_FACTOR = 0.01; // The factor to multiply driver input by when changing the offset.
    private static final boolean IS_MANUAL = false;

    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Tilted Elevator");
    private final GenericEntry extensionPEntry = shuffleboardTab.add("Extension P", extensionP).getEntry();
    private final GenericEntry extensionIEntry = shuffleboardTab.add("Extension I", extensionI).getEntry();
    private final GenericEntry extensionDEntry = shuffleboardTab.add("Extension D", extensionD).getEntry();
    private final GenericEntry extensionFFEntry = shuffleboardTab.add("Extension FF", extensionFF).getEntry();
    private final GenericEntry extensionToleranceEntry = shuffleboardTab.add("Extension Tolerance", extensionTolerance).getEntry();
    private final GenericEntry arbFFEntry = shuffleboardTab.add("Arb FF", arbFeedforward).getEntry();

    private final GenericEntry manualPowerEntry = shuffleboardTab.add("Manual Power", manualPower).getEntry();

    private final GenericEntry maxVelEntry = shuffleboardTab.add("Max Vel", maxVel).getEntry();
    private final GenericEntry maxAccelEntry = shuffleboardTab.add("Max Accel", maxAccel).getEntry();
    private final GenericEntry currentVelEntry = shuffleboardTab.add("Current Vel (mps)", 0.0).getEntry();

    private final GenericEntry targetExtensionEntry = shuffleboardTab.add("Target Ext (in)", 0.0).getEntry();
    private final GenericEntry currentExtensionEntry = shuffleboardTab.add("Current Ext (in)", 0.0).getEntry();
    private final GenericEntry currentStateEntry = shuffleboardTab.add("Current State", currentState.toString()).getEntry();

    private final GenericEntry offsetDistEntry = shuffleboardTab.add("offset (in)", offsetDistMeters).getEntry();

    public enum ElevatorState {
        GROUND(0), // retracted
        CHUTE(Units.inchesToMeters(40)),
        SUBSTATION(Units.inchesToMeters(50)), // absolute height = 37.375 in
        CUBE_MID(Units.inchesToMeters(33)), // absolute height = 14.25 in
        CUBE_HIGH(Units.inchesToMeters(53)), // absolute height = 31.625 in
        CONE_MID(Units.inchesToMeters(50)), // absolute height = 34 in
        CONE_HIGH(Units.inchesToMeters(0)); // absolute height = 46 in

        public double extendDistanceMeters; // meters, extension distance of winch

        private static final double CUBE_OFFSET = Units.inchesToMeters(0); // meters, intake jaw to carriage bottom
        private static final double CONE_OFFSET = Units.inchesToMeters(0); // meters, intake jaw to carriage bottom

        /**
         * ElevatorState defined by extension of elevator from zero. All values in meters and radians.
         * @param extendDistanceMeters The extension of the subsystem.
         */
        private ElevatorState(double extendDistanceMeters) {
            this.extendDistanceMeters = extendDistanceMeters;
        }
    }

    public TiltedElevatorSubsystem() {
        extensionMotor = MotorUtil.createSparkMax(EXTENSION_ID);
        extensionMotor.setIdleMode(IdleMode.kBrake); 
        extensionMotor.setInverted(true);

        extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        extensionMotor.setSoftLimit(SoftLimitDirection.kForward, EXTENSION_LIMIT);
        extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.inchesToMeters(-2));

        extensionEncoder = extensionMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(EXTENSION_ROTATIONS_TO_METERS);
        extensionEncoder.setVelocityConversionFactor(EXTENSION_ROTATIONS_TO_METERS / 60.0);
        extensionEncoder.setPosition(0);

        extensionPidController = extensionMotor.getPIDController();
        extensionPidController.setP(extensionP);
        extensionPidController.setI(extensionI);
        extensionPidController.setD(extensionD);
        extensionPidController.setFF(extensionFF);
        extensionPidController.setSmartMotionAllowedClosedLoopError(extensionTolerance, 0);
        extensionPidController.setSmartMotionMaxVelocity(maxVel, 0);
        extensionPidController.setSmartMotionMaxAccel(maxAccel, 0);
        extensionPidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

        extensionFollow = MotorUtil.createSparkMax(EXTENSION_FOLLOW_ID);
        extensionFollow.follow(extensionMotor);
        extensionFollow.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        if (zeroLimitSwitch.get()) extensionEncoder.setPosition(0);

        if (IS_MANUAL) {
            manualPowerEntry.setDouble(manualPower);
            extensionMotor.set(manualPower);
            return;
        }

        // Otherwise, use PID
        extensionPidController.setP(extensionPEntry.getDouble(extensionP));
        extensionPidController.setI(extensionIEntry.getDouble(extensionI));
        extensionPidController.setD(extensionDEntry.getDouble(extensionD));
        extensionPidController.setFF(extensionFFEntry.getDouble(extensionFF));
        extensionPidController.setSmartMotionMaxVelocity(maxVelEntry.getDouble(maxVel), 0);
        extensionPidController.setSmartMotionMaxAccel(maxAccelEntry.getDouble(maxAccel), 0);
        extensionPidController.setSmartMotionAllowedClosedLoopError(extensionToleranceEntry.getDouble(extensionTolerance), 0);
        arbFeedforward = arbFFEntry.getDouble(arbFeedforward);
        

        // System.out.println(extensionEncoder.getPosition());

        double currentPos = extensionEncoder.getPosition();
        double currentVel = extensionEncoder.getVelocity();

        currentVelEntry.setDouble(currentVel);
        currentExtensionEntry.setDouble(Units.metersToInches(currentPos));
        offsetDistEntry.setDouble(Units.metersToInches(offsetDistMeters));

        // Units.inchesToMeters(targetExtensionEntry.getDouble(0));
        this.targetExtension = currentState.extendDistanceMeters + offsetDistMeters;

        // give up 
        if (this.targetExtension == 0 && currentPos < Units.inchesToMeters(1)) {
            extensionMotor.set(0);
        } else if (this.targetExtension == 0 && currentPos < Units.inchesToMeters(5)) {
            // bring down to lim switch
            extensionMotor.set(-0.075);
        } else {
            // Set PID reference
            extensionPidController.setReference(MathUtil.clamp(targetExtension, 0, EXTENSION_LIMIT), ControlType.kSmartMotion, 0, arbFeedforward, ArbFFUnits.kPercentOut);
        }

        currentStateEntry.setString(currentState.toString());
    }

    /**
     * Toggles the state of the subsystem between the two specified `ElevatorState`s, defaulting
     * to state 1. Also resets the distance offset.
     * 
     * @param state1 The first state.
     * @param state2 The second state.
     */
    public void toggleState(ElevatorState state1, ElevatorState state2) {
        resetOffset();
        currentState = currentState == state1
            ? state2
            : state1;
    }

    /**
     * Changes the distance offset from given driver powers.
     * @param power The power to change the offset by.
     */
    public void changeOffsetDistMeters(double power) {
        this.offsetDistMeters += OFFSET_FACTOR * power;
    }

    /**
     * Zeros the distance offset.
     */
    public void resetOffset() {
        this.offsetDistMeters = 0;
    }

    /**
     * Sets the manual power of this subsystem.
     * @param power Xbox-controlled power
     */
    public void setManualPower(double power) {
        if (power > 0.5) {
            this.manualPower = 0.3;
        } else if (manualPower < -0.5) {
            this.manualPower = -0.3;
        } else {
            this.manualPower = 0;
        }
    }
}
