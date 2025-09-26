package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
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
import frc.robot.Constants.AlgaePivotConstants;

public class AlgaePivotSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig;
    private final ProfiledPIDController pid;
    private final ArmFeedforward feedforward;
    private final DutyCycleEncoder encoder;
    private final MutAngle algaeAngle;
    private final Trigger loopEnabled;

    // sysid specific variables
    private final RelativeEncoder pivotEncoder;
    private final SysIdRoutine sysIdRoutine;
    private final MutAngularVelocity angularVelocity = DegreesPerSecond.mutable(0);
    private final MutVoltage voltsDrawn = Volts.mutable(0);

    public AlgaePivotSubsystem() {
        pid = new ProfiledPIDController(AlgaePivotConstants.kP,
                AlgaePivotConstants.kI,
                AlgaePivotConstants.kD,
                new Constraints(AlgaePivotConstants.MAX_VELOCITY.in(RadiansPerSecond),
                        AlgaePivotConstants.MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
        feedforward = new ArmFeedforward(AlgaePivotConstants.Ks, AlgaePivotConstants.Kg, AlgaePivotConstants.Kv);
        motor = new SparkMax(AlgaePivotConstants.PIVOT_ID, MotorType.kBrushless);
        pivotEncoder = motor.getEncoder();
        motorConfig = new SparkMaxConfig();
        encoder = new DutyCycleEncoder(AlgaePivotConstants.ENCODER_ID);
        loopEnabled = new Trigger(() -> SmartDashboard.getBoolean("AlgaePivot/Enabled", true));
        algaeAngle = Degrees.mutable(0);

        setGoal(Degrees.of(85));
        pid.setTolerance(Degrees.of(4).in(Radians));

        // configure sparkmaxes
        motorConfig
                .smartCurrentLimit(AlgaePivotConstants.CURRENT_LIMIT)
                .idleMode(AlgaePivotConstants.IDLE_MODE)
                .inverted(AlgaePivotConstants.INVERTED);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Diagnostic disable
        SmartDashboard.putBoolean("AlgaePivot/Enabled", true);
        loopEnabled.and(RobotState::isEnabled).whileTrue(updateAlgaeControl());

        // tuning
        Preferences.initDouble("AlgaePivot/Kg", 0);
        Preferences.initDouble("AlgaePivot/Kv", 0);
        Preferences.initDouble("AlgaePivot/Target Pref Degrees", 0);
        SmartDashboard.putData("AlgaePivot/Update Target Goal", updateTargetPreference());
        SmartDashboard.putData("AlgaePivot/PID", pid);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(1).per(Second),
                        Volt.of(3.5),
                        Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        motor::setVoltage,
                        log -> {
                            log.motor("algae")
                                    .voltage(voltsDrawn.mut_replace(motor.getAppliedOutput() * motor.getBusVoltage(),
                                            Volts))
                                    .angularPosition(getAlgaeAngle())
                                    .angularVelocity(getAlgaeAngularVelocity());
                        }, this));
    }

    /**
     * Updates the coral's PID and FF terms to track a requested position. Will set
     * motor voltage to zero volts when interrupted.
     * 
     * @return the command to run the pivot motor
     */
    private Command updateAlgaeControl() {
        return Commands // does not interrupt other commands
                .startRun(
                        () -> pid.reset(getAlgaeAngle().in(Radians)),
                        () -> {
                            double pideffort = pid.calculate(getAlgaeAngle().in(Radians));
                            TrapezoidProfile.State state = pid.getSetpoint();
                            runPivot(pideffort + feedforward.calculate(state.position, state.velocity));
                        })
                .withName("AlgaePivotControlLoop")
                .finallyDo(() -> runPivot(0));
    }

    /**
     * Moves the algae to a requested angle. Will wait until it has reached the
     * angle to end the command, along with a 0.25s debounce.
     * 
     * @param angle the requested angle to move the algae to.
     * @return the command to move the algae angle
     */
    public Command moveAlgaeCommand(Angle angle) {
        return this
                .runOnce(() -> this.setGoal(angle))
                .andThen(Commands.waitUntil(this::isAtPosition))
                .andThen(Commands.waitSeconds(0.1)).withName("AlgaeMoving");
    }

    /**
     * Sets the algae target angle.
     * 
     * @param angle the target angle
     */
    private void setGoal(Angle angle) {
        SmartDashboard.putNumber("AlgaePivot/Target", angle.in(Degrees));
        pid.setGoal(angle.in(Radians));
    }

    /**
     * Runs the pivot motor.
     * 
     * @param volts voltage to run the pivot motor at
     */
    private void runPivot(double volts) {
        motor.setVoltage(volts);
    }

    /**
     * Updates the target with the target from the preferences.
     * 
     * @return the command to update the target
     */
    private Command updateTargetPreference() {
        return this.runOnce(
                () -> setGoal(Degrees.of(Preferences.getDouble("AlgaePivot/Target Pref Degrees", 0.0))));
    }

    /**
     * @return The algae's rotation from the horizon.
     */
    private Angle getAlgaeAngle() {
        algaeAngle.mut_replace(encoder.get(), Rotations);
        return algaeAngle.mut_minus(AlgaePivotConstants.ENCODER_OFFSET);
    }

    private AngularVelocity getAlgaeAngularVelocity() {
        return angularVelocity.mut_replace(pivotEncoder.getVelocity() / AlgaePivotConstants.GEAR_RATIO, RPM);
    }

    /**
     * @return Whether the algae has reached the target angle.
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
        // tuning
        // feedforward.setKg(Preferences.getDouble("AlgaePivot/Kg", 0));
        // feedforward.setKv(Preferences.getDouble("AlgaePivot/Kv", 0));

        SmartDashboard.putNumber("AlgaePivot/Angle (Degrees)", getAlgaeAngle().in(Degrees));
        SmartDashboard.putBoolean("AlgaePivot/At Position", isAtPosition());
    }
}
