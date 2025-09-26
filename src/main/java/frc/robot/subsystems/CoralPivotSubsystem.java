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
import frc.robot.Constants.CoralPivotConstants;

public class CoralPivotSubsystem extends SubsystemBase {
    private final ProfiledPIDController pid;
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig;
    private final DutyCycleEncoder encoder;
    private final Trigger coralPIDEnabled;
    private final MutAngle coralAngle;
    private final ArmFeedforward coralFeedforward;

    // required for characterization
    private final MutVoltage voltsDrawn = Volts.mutable(0);
    private final MutAngularVelocity angularVelocity = RPM.mutable(0);
    private final RelativeEncoder pivotEncoder;

    private final SysIdRoutine sysIdRoutine;

    public CoralPivotSubsystem() {
        pid = new ProfiledPIDController(CoralPivotConstants.kP, CoralPivotConstants.kI, CoralPivotConstants.kD,
                new Constraints(CoralPivotConstants.MAX_VELOCITY.in(RadiansPerSecond),
                        CoralPivotConstants.MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
        coralFeedforward = new ArmFeedforward(CoralPivotConstants.Ks, CoralPivotConstants.Kg, CoralPivotConstants.Kv,
                CoralPivotConstants.Ka);

        motor = new SparkMax(CoralPivotConstants.MOTOR_ID, MotorType.kBrushless);
        pivotEncoder = motor.getEncoder();
        motorConfig = new SparkMaxConfig();
        encoder = new DutyCycleEncoder(CoralPivotConstants.ENCODER_ID);
        coralPIDEnabled = new Trigger(() -> SmartDashboard.getBoolean("CoralPivot/PID Enabled", true));
        coralAngle = Degrees.mutable(0);

        // configure PID
        pid.setTolerance(Degrees.of(2).in(Radians));
        setGoal(CoralPivotConstants.STOWED);

        pivotEncoder.setPosition(getCoralAngle().in(Rotations));

        // configure sparkmaxes
        motorConfig
                .smartCurrentLimit(CoralPivotConstants.CURRENT_LIMIT)
                .inverted(CoralPivotConstants.INVERTED)
                .idleMode(CoralPivotConstants.IDLE_MODE);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Diagnostic stuff
        SmartDashboard.putBoolean("CoralPivot/PID Enabled", true);
        coralPIDEnabled.and(RobotState::isEnabled).whileTrue(updateAngleControl());

        // everything below can be commented out once tuned
        Preferences.initDouble("CoralPivot/Kg", 0.0);
        Preferences.initDouble("CoralPivot/Kv", 0.0);
        Preferences.initDouble("CoralPivot/Target Pref Degrees", 0.0);
        SmartDashboard.putData("CoralPivot/Move From Preferences Degrees", updatePref());
        SmartDashboard.putData("CoralPivot/PID", pid);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(1).per(Second),
                        Volt.of(2),
                        Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        motor::setVoltage,
                        log -> {
                            log.motor("coral")
                                    .voltage(voltsDrawn.mut_replace(motor.getAppliedOutput() * motor.getBusVoltage(),
                                            Volts))
                                    .angularPosition(getCoralAngle())
                                    .angularVelocity(getCoralAngularVelocity());
                        }, this));
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
        return Commands // does not interrupt other commands
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
                .withName("CoralPivotControlLoop")
                // if disabled, set the voltage to zero
                .finallyDo(() -> runPivot(0));
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
     * Sets the coral's target angle. The coral's pivot will rotate to that angle
     * when the {@code updateAngleControl() } command is active.
     * 
     * @param angle the arm's angle to rotate to.
     */
    private void setGoal(Angle angle) {
        // check to make sure it isn't being set above or below the max/min limits
        if (angle.lt(CoralPivotConstants.UPPER_LIMIT) && angle.gt(CoralPivotConstants.LOWER_LIMIT)) {
            SmartDashboard.putNumber("CoralPivot/Target", angle.in(Degrees));
            pid.setGoal(angle.in(Radians));
        }
    }

    /**
     * Runs the pivot motor.
     * 
     * @param volts voltage to set the motor to
     */
    private void runPivot(double volts) {
        motor.setVoltage(volts);
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
                        Degrees.of(Preferences.getDouble("CoralPivot/Target Pref Degrees", 0.0))));
    }

    /**
     * @return The coral angle from the ground
     */
    private Angle getCoralAngle() {
        coralAngle.mut_replace(encoder.get(), Rotations);
        return coralAngle.mut_minus(CoralPivotConstants.ENCODER_OFFSET);
    }

    private Angle getcoralAngleSpark() {
        return Rotations.of(pivotEncoder.getPosition() / CoralPivotConstants.GEAR_RATIO);
    }

    private AngularVelocity getCoralAngularVelocity() {
        return angularVelocity.mut_replace(pivotEncoder.getVelocity() / CoralPivotConstants.GEAR_RATIO, RPM);
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
        // coralFeedforward.setKg(Preferences.getDouble("CoralPivot/Kg", 0.0));
        // coralFeedforward.setKv(Preferences.getDouble("CoralPivot/Kv", 0.0));

        SmartDashboard.putNumber("CoralPivot/Position Angle", getCoralAngle().in(Degrees));
        SmartDashboard.putBoolean("CoralPivot/At Position", isAtPosition());
        SmartDashboard.putNumber("CoralPivot/Angle Spark", getcoralAngleSpark().in(Degrees));
    }
}
