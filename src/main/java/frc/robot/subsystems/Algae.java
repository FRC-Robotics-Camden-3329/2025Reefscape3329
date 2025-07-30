package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
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
import frc.robot.Constants.AlgaeConstants;

public class Algae extends SubsystemBase {
    private SparkMax pivot, intake;
    private RelativeEncoder pivotEncoder;
    private SparkMaxConfig config;
    private ProfiledPIDController pid;
    private ArmFeedforward feedforward;
    private DutyCycleEncoder encoder;
    private MutAngle algaeAngle;
    private Trigger loopEnabled;
    private Trigger algaeDetected;
    private SysIdRoutine sysIdRoutine;
    private MutAngularVelocity angularVelocity = DegreesPerSecond.mutable(0);
    private MutVoltage voltsDrawn = Volts.mutable(0);

    public Algae() {
        pid = new ProfiledPIDController(AlgaeConstants.kP,
                AlgaeConstants.kI,
                AlgaeConstants.kD,
                new Constraints(AlgaeConstants.MAX_VELOCITY.in(RadiansPerSecond),
                        AlgaeConstants.MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
        feedforward = new ArmFeedforward(AlgaeConstants.Ks, AlgaeConstants.Kg, AlgaeConstants.Kv);
        pivot = new SparkMax(AlgaeConstants.pivotID, MotorType.kBrushless);
        pivotEncoder = pivot.getEncoder();
        intake = new SparkMax(AlgaeConstants.intakeID, MotorType.kBrushed);
        config = new SparkMaxConfig();
        encoder = new DutyCycleEncoder(AlgaeConstants.encoderID);
        algaeDetected = new Trigger(() -> intake.getOutputCurrent() > 30).debounce(0.8);
        loopEnabled = new Trigger(() -> SmartDashboard.getBoolean("Algae/Enabled", true));
        algaeAngle = Degrees.mutable(0);

        setTarget(Degrees.of(85));
        pid.setTolerance(Degrees.of(4).in(Radians));

        // configure sparkmaxes
        config
                .smartCurrentLimit(40) // 40 Amps, intake should probable be less than pivot
                .idleMode(IdleMode.kBrake)
                .inverted(true); // they are both inverted
        intake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Diagnostic disable
        SmartDashboard.putBoolean("Algae/Enabled", true);
        loopEnabled.and(RobotState::isEnabled).whileTrue(updateCoralControl());

        // tuning
        Preferences.initDouble("Algae/Kg", 0);
        Preferences.initDouble("Algae/Kv", 0);
        Preferences.initDouble("Algae/Target Pref Degrees", 0);
        SmartDashboard.putData("Algae/Update Target Goal", updateTargetPreference());
        SmartDashboard.putData("Algae/PID", pid);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(1).per(Second),
                        Volt.of(3.5),
                        Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        pivot::setVoltage,
                        log -> {
                            log.motor("algae")
                                    .voltage(voltsDrawn.mut_replace(pivot.getAppliedOutput() * pivot.getBusVoltage(),
                                            Volts))
                                    .angularPosition(getAlgaeAngle())
                                    .angularVelocity(getAlgaeAngularVelocity());
                        }, this));
    }

    /**
     * Updates the target with the target from the preferences.
     * 
     * @return the command to update the target
     */
    private Command updateTargetPreference() {
        return this.runOnce(
                () -> setTarget(Degrees.of(Preferences.getDouble("Algae/Target Pref Degrees", 0.0))));
    }

    /**
     * Updates the coral's PID and FF terms to track a requested position. Will set
     * motor voltage to zero volts when interrupted.
     * 
     * @return the command to run the pivot motor
     */
    private Command updateCoralControl() {
        return Commands
                .startRun(
                        () -> pid.reset(getAlgaeAngle().in(Radians)),
                        () -> {
                            double pideffort = pid.calculate(getAlgaeAngle().in(Radians));
                            TrapezoidProfile.State state = pid.getSetpoint();
                            runPivot(pideffort + feedforward.calculate(state.position, state.velocity));
                        })
                .finallyDo(() -> runPivot(0));
    }

    /**
     * Runs the pivot motor.
     * 
     * @param volts voltage to run the pivot motor at
     */
    private void runPivot(double volts) {
        pivot.setVoltage(volts);
    }

    /**
     * Sets the algae target angle.
     * 
     * @param angle the target angle
     */
    private void setTarget(Angle angle) {
        SmartDashboard.putNumber("Algae/Target", angle.in(Degrees));
        pid.setGoal(angle.in(Radians));
    }

    /**
     * @return The algae's rotation from the horizon.
     */
    private Angle getAlgaeAngle() {
        algaeAngle.mut_replace(encoder.get(), Rotations);
        return algaeAngle.mut_minus(AlgaeConstants.ENCODER_OFFSET);
    }

    private AngularVelocity getAlgaeAngularVelocity() {
        return angularVelocity.mut_replace(pivotEncoder.getVelocity() / AlgaeConstants.GEAR_RATIO, RPM);
    }

    /**
     * @return Whether the algae has reached the target angle.
     */
    private boolean isAtPosition() {
        return pid.atGoal();
    }

    /**
     * @return Gets whether algae has been detected as intaked.
     */
    public Trigger getAlgaeDetected() {
        return algaeDetected;
    }

    /**
     * Moves the algae to a requested angle. Will wait until it has reached the
     * angle to end the command, along with a 0.25s debounce.
     * 
     * @param angle the requested angle to move the algae to.
     * @return the command to move the algae angle
     */
    public Command moveAlgaeCommand(Angle angle) {
        return Commands // should probably be `this`
                .runOnce(() -> this.setTarget(angle))
                .andThen(Commands.waitUntil(this::isAtPosition))
                .andThen(Commands.waitSeconds(0.1));
    }

    /**
     * Runs the intake at a particular speed.
     * 
     * @param speed a percentage from [-1, 1].
     */
    private void runIntake(double speed) {
        // setting voltage is more repeatable, so we convert from percentage to voltage
        // assuming a max voltage of 12V
        intake.setVoltage(speed * 12.0);
    }

    /**
     * Intakes algae. Will end the command when algae has been detected as intaked.
     * After ending, the intake motors will either turn off if there is no algae
     * detected or contine running inwards to grip algae if algae is detected.
     * 
     * @return command to intake algae
     */
    public Command intakeAlgaeCommand() {
        return this
                .run(() -> this.runIntake(AlgaeConstants.INTAKE_SPEED))
                .until(algaeDetected)
                .finallyDo(() -> {
                    if (algaeDetected.getAsBoolean()) {
                        runIntake(AlgaeConstants.HOLD_SPEED);
                    } else {
                        runIntake(0.0);
                    }
                });
    }

    /**
     * Command to eject algae. Will run for 0.5 seconds and then end.
     * 
     * @return command to eject algae
     */
    public Command ejectAlgaeCommand() {
        return this.runEnd(
                () -> this.runIntake(-AlgaeConstants.EJECT_SPEED),
                () -> this.runIntake(0))
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
        // tuning
        // feedforward.setKg(Preferences.getDouble("Algae/Kg", 0));
        // feedforward.setKv(Preferences.getDouble("Algae/Kv", 0));

        SmartDashboard.putBoolean("Algae/Detected", algaeDetected.getAsBoolean());
        SmartDashboard.putNumber("Algae/Angle (Degrees)", getAlgaeAngle().in(Degrees));
        SmartDashboard.putNumber("Algae/Intake Current (A)", intake.getOutputCurrent());
        SmartDashboard.putBoolean("Algae/At Position", isAtPosition());
    }
}
