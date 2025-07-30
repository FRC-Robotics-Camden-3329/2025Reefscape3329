package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CoralConstants;

public class Coral extends SubsystemBase {
    private ProfiledPIDController pid;
    private SparkMax pivot, intake;
    private SparkMaxConfig intakeConfig;
    private DutyCycleEncoder encoder;
    private Trigger coralDetected;
    private Trigger coralPIDEnabled;
    private MutAngle coralAngle;
    private ArmFeedforward coralFeedforward;
    private double coralSetpoint;

    // required for characterization
    private MutVoltage voltsDrawn = Volts.mutable(0);
    private MutAngularVelocity angularVelocity = RPM.mutable(0);
    private RelativeEncoder pivotEncoder;

    private SysIdRoutine sysIdRoutine;

    public Coral() {
        pid = new ProfiledPIDController(CoralConstants.kP, CoralConstants.kI, CoralConstants.kD,
                new Constraints(CoralConstants.MAX_VELOCITY.in(RadiansPerSecond),
                        CoralConstants.MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
        coralFeedforward = new ArmFeedforward(CoralConstants.Ks, CoralConstants.Kg, CoralConstants.Kv,
                CoralConstants.Ka);

        pivot = new SparkMax(CoralConstants.pivotID, MotorType.kBrushless);
        pivotEncoder = pivot.getEncoder();
        intake = new SparkMax(CoralConstants.intakeID, MotorType.kBrushless);
        intakeConfig = new SparkMaxConfig();
        encoder = new DutyCycleEncoder(CoralConstants.encoderID);
        coralDetected = new Trigger(() -> intake.getOutputCurrent() > 10 /* Amps */).debounce(0.15);
        coralPIDEnabled = new Trigger(() -> SmartDashboard.getBoolean("Coral/PID Enabled", true));
        coralAngle = Degrees.mutable(0);

        // configure PID
        pid.setTolerance(Degrees.of(2).in(Radians));
        setGoal(CoralConstants.STOWED);

        pivotEncoder.setPosition(getCoralAngle().in(Rotations));

        // configure sparkmaxes
        intakeConfig
                .smartCurrentLimit(40) // 40 Amps
                .idleMode(IdleMode.kBrake);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivot.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Diagnostic stuff
        SmartDashboard.putBoolean("Coral/PID Enabled", true);
        coralPIDEnabled.and(RobotState::isEnabled).whileTrue(updateAngleControl());

        // everything below can be commented out once tuned
        Preferences.initDouble("Coral/Kg", 0.0);
        Preferences.initDouble("Coral/Kv", 0.0);
        Preferences.initDouble("Coral/Target Pref Degrees", 0.0);
        SmartDashboard.putData("Coral/Move From Preferences Degrees", updatePref());
        SmartDashboard.putData("Coral/PID", pid);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(1).per(Second),
                        Volt.of(2),
                        Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        pivot::setVoltage,
                        log -> {
                            log.motor("coral")
                                    .voltage(voltsDrawn.mut_replace(pivot.getAppliedOutput() * pivot.getBusVoltage(),
                                            Volts))
                                    .angularPosition(getCoralAngle())
                                    .angularVelocity(getCoralAngularVelocity());
                        }, this));
    }

    /**
     * Runs the pivot motor.
     * 
     * @param volts voltage to set the motor to
     */
    private void runPivot(double volts) {
        pivot.setVoltage(volts);
    }

    /**
     * Updates the coral setpoint based on the current preferences value. Only used
     * for diagnostic/tuning purposes.
     * 
     * @return the command to update the coral setpoint
     */
    private Command updatePref() {
        return Commands.runOnce(
                () -> setGoal(
                        Degrees.of(Preferences.getDouble("Coral/Target Pref Degrees", coralSetpoint))));
    }

    /**
     * Sets the coral's target angle. The coral's pivot will rotate to that angle
     * when the {@code updateAngleControl() } command is active.
     * 
     * @param angle the arm's angle to rotate to.
     */
    private void setGoal(Angle angle) {
        // check to make sure it isn't being set above or below the max/min limits
        if (angle.lt(CoralConstants.UPPER_LIMIT) && angle.gt(CoralConstants.LOWER_LIMIT)) {
            SmartDashboard.putNumber("Coral/Target", angle.in(Degrees));
            pid.setGoal(angle.in(Radians));
        }
    }

    /**
     * @return The coral angle from the ground
     */
    private Angle getCoralAngle() {
        coralAngle.mut_replace(encoder.get(), Rotations);
        return coralAngle.mut_minus(CoralConstants.ENCODER_OFFSET);
    }

    private Angle getcoralAngleSpark() {
        return Rotations.of(pivotEncoder.getPosition() / CoralConstants.GEAR_RATIO);
    }

    private AngularVelocity getCoralAngularVelocity() {
        return angularVelocity.mut_replace(pivotEncoder.getVelocity() / CoralConstants.GEAR_RATIO, RPM);
    }

    /**
     * Whether the coral has reached it's final destination.
     * 
     * @return true if reached, false if not
     */
    private boolean isAtPosition() {
        return pid.atGoal();
    }

    /**
     * Moves the coral's pivot to a specified degree amount. The command does not
     * end until the requested angle has been reached. Useful for command chaining.
     * 
     * @param angle pivot angle from the ground
     * @return the command to move the coral
     */
    public Command moveCoralCommand(Angle angle) {
        return this
                .runOnce(() -> setGoal(angle))
                .andThen(Commands.waitUntil(this::isAtPosition))
                .andThen(Commands.waitSeconds(0.1));
    }

    /**
     * Updates the coral's pivot motor's PID loop and feedforward term. When
     * running, the PID loop is updated. When ended, power to the pivot motor is set
     * to 0.
     * <p>
     * Left as {@code true} during competition. Use as a diagnostic tool if needed.
     * 
     * @return the command to update the pivot's PID
     */
    private Command updateAngleControl() {
        return Commands
                .startRun(
                        // on start, update the pid's inital value
                        () -> pid.reset(getCoralAngle().in(Radians)),
                        () -> {
                            // continously update both the pid and ff
                            double pideffort = pid.calculate(getCoralAngle().in(Radians)); // also updates setpoint
                            TrapezoidProfile.State currState = pid.getSetpoint();
                            runPivot(pideffort
                                    + coralFeedforward.calculate(currState.position, currState.velocity));
                        })
                // if disabled, set the voltage to zero
                .finallyDo(() -> runPivot(0));
    }

    /**
     * Gets whether the coral is detected.
     * 
     * @return the trigger to use off coral detection
     */
    public Trigger getCoralDetected() {
        return coralDetected;
    }

    /**
     * Runs the intake
     * 
     * @param speed set speed from [-1,1]
     */
    private void runIntake(double speed) {
        // setting voltage is more repeatable, so we convert from percentage to voltage
        // assuming a max voltage of 12V
        intake.setVoltage(-speed * 12.0);
    }

    /**
     * Runs the intake. Will run the intake motors backwards to hold onto the coral
     * if the game piece is coral was detected. Will set the motors to zero if not
     * detected.
     * 
     * @return run intake command
     */
    public Command intakeCoralCommand() {
        return this // intake
                .run(() -> runIntake(CoralConstants.intakeSpeed))
                // stop intaking when coral detected
                .until(coralDetected)
                .finallyDo(interrupted -> {
                    // if holding coral, run intake for better grip, otherwise turn off the intake
                    if (coralDetected.getAsBoolean()) {
                        runIntake(0.05);
                    } else {
                        runIntake(0.0);
                    }
                });
    }

    /**
     * Ejects the coral. Command ends after 0.5 seconds.
     * 
     * @return eject coral command
     */
    public Command ejectCoralCommand() {
        return this.runEnd(
                () -> runIntake(-CoralConstants.ejectSpeed),
                () -> runIntake(0))
                // run the intake for half a second for ejceting coral
                .withTimeout(0.5);
    }

    /**
     * Ejects the coral. Command ends after 0.5 seconds.
     * 
     * @return eject coral command
     */
    public Command ejectCoralSlowCommand() {
        return this.runEnd(
                () -> runIntake(-CoralConstants.ejectSpeed / 2.0),
                () -> runIntake(0))
                // run the intake for half a second for ejceting coral
                .withTimeout(0.5);
    }

    /**
     * Sysid command for running the quasistatic characterization routine.
     * 
     * @param direction the direction to run the characterization routine
     * @return the sysid routine
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Sysid command for running the dynamic characterization routine.
     * 
     * @param direction the direction to run the characterization routine
     * @return the sysid routine
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        // update the kg term based on preferences, can be commented once calibrated
        // coralFeedforward.setKg(Preferences.getDouble("Coral/Kg", 0.0));
        // coralFeedforward.setKv(Preferences.getDouble("Coral/Kv", 0.0));

        SmartDashboard.putNumber("Coral/Position Angle", getCoralAngle().in(Degrees));
        SmartDashboard.putBoolean("Coral/At Position", isAtPosition());
        SmartDashboard.putNumber("Coral/Angle Spark", getcoralAngleSpark().in(Degrees));
        SmartDashboard.putNumber("Coral/Intake Current (A)", intake.getOutputCurrent());
        SmartDashboard.putBoolean("Coral/Detected", getCoralDetected().getAsBoolean());
    }
}
