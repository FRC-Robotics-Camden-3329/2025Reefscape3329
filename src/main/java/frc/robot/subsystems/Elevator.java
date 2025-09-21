package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private SparkMax left, right;
    private SparkMaxConfig leftMotorConfig;
    private SparkMaxConfig rightMotorConfig;
    private RelativeEncoder encoder;
    private MutDistance elevatorHeight;
    private ProfiledPIDController pid;
    private ElevatorFeedforward feedforward;
    private Trigger loopEnabled;
    private MutLinearVelocity elevatorLinearVelocity;
    private MutVoltage elevatorVoltage;

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    volts -> {
                        right.setVoltage(volts);
                        left.setVoltage(volts);
                    },
                    log -> {
                        log.motor("elevator")
                                .voltage(elevatorVoltage.mut_replace(left.getAppliedOutput() * left.getBusVoltage(),
                                        Volts))
                                .linearPosition(getElevatorHeight())
                                .linearVelocity(getElevatorVelocity());
                    }, this));

    public Elevator() {
        right = new SparkMax(ElevatorConstants.rightID, MotorType.kBrushless);
        left = new SparkMax(ElevatorConstants.leftID, MotorType.kBrushless);
        leftMotorConfig = new SparkMaxConfig();
        rightMotorConfig = new SparkMaxConfig();
        pid = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
                new Constraints(ElevatorConstants.MAX_VELOCITY.in(MetersPerSecond),
                        ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond)));
        feedforward = new ElevatorFeedforward(ElevatorConstants.Ks, ElevatorConstants.Kg, ElevatorConstants.Kv,
                ElevatorConstants.Ka);
        loopEnabled = new Trigger(() -> SmartDashboard.getBoolean("Elevator/Enabled", true));
        elevatorHeight = Inches.mutable(0);
        elevatorVoltage = Volts.mutable(0);
        elevatorLinearVelocity = FeetPerSecond.mutable(0);

        encoder = left.getEncoder();
        encoder.setPosition(0);

        pid.setTolerance(Inches.of(2).in(Meters));

        rightMotorConfig
                .smartCurrentLimit((int) ElevatorConstants.MOTOR_CURRENT_LIMIT.in(Amps))
                .idleMode(IdleMode.kBrake)
                .inverted(true);
        right.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotorConfig
                .smartCurrentLimit((int) ElevatorConstants.MOTOR_CURRENT_LIMIT.in(Amps))
                .idleMode(IdleMode.kBrake);
        left.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // the elevator starts at the floor offset height
        setTargetHeight(ElevatorConstants.FLOOR_OFFSET);

        // Diagnostic disable
        SmartDashboard.putBoolean("Elevator/Enabled", true);
        loopEnabled.and(RobotState::isEnabled).whileTrue(updateElevatorControl());

        // Automatically disable the PID if the current is too high for too long. Also
        // sets the target height to be at the floor so it doesn't jolt back up if
        // manually reenabled.
        new Trigger(() -> left.getOutputCurrent() >= (ElevatorConstants.MOTOR_CURRENT_LIMIT.in(Amps) - 1.0))
                .debounce(1.0) // should be tuned to not cause any false positives during normal operation
                .onTrue(Commands.runOnce(() -> {
                    SmartDashboard.putBoolean("Elevator/Enabled", false);
                    setTargetHeight(ElevatorConstants.FLOOR_OFFSET);
                    DriverStation.reportError("Elevator PID disabled due to excessive current draw!!", false);
                }));

        // tuning
        Preferences.initDouble("Elevator/Kg", 0);
        Preferences.initDouble("Elevator/Kv", 0);
        Preferences.initDouble("Elevator/Target Height (in)", ElevatorConstants.FLOOR_OFFSET.in(Inches));
        SmartDashboard.putData("Elevator/Update from preferences", updateFromPreferences());
        SmartDashboard.putData("Elevator/PID", pid);
    }

    /**
     * @return Command to sync the target height preferences.
     */
    private Command updateFromPreferences() {
        return this.runOnce(() -> setTargetHeight(Inches
                .of(Preferences.getDouble("Elevator/Target Height (in)", ElevatorConstants.FLOOR_OFFSET.in(Inches)))));
    }

    /**
     * @return The command to update both the PID and FF terms for the elevator
     *         motors.
     */
    private Command updateElevatorControl() {
        return Commands.startRun(
                // first reset the pid to the current height
                () -> pid.reset(getElevatorHeight().in(Meters)),
                // on each loop, run the elevator with the current pid and ff
                () -> runElevator(pid.calculate(getElevatorHeight().in(Meters))
                        + feedforward.calculate(pid.getSetpoint().velocity)))
                // if interrupted, set the elevator motors to zero volts
                .finallyDo(() -> runElevator(0));
    }

    /**
     * Runs the elevator motors.
     * 
     * @param volts requested volts to run the motors off of
     */
    private void runElevator(double volts) {
        right.setVoltage(volts);
        left.setVoltage(volts);
    }

    /**
     * Sets the target height and adds it to the dashboard.
     * 
     * @param height requested elevator height. Zero is the ground and
     *               positive is up.
     */
    private void setTargetHeight(Distance height) {
        SmartDashboard.putNumber("Elevator/Target Height (in)", height.in(Inches));
        pid.setGoal(height.in(Meters));
    }

    /**
     * Gets the elevator velocity. Used for characterization purposes.
     * 
     * @return the elevator's velocity
     */
    private LinearVelocity getElevatorVelocity() {
        // manually convert from per minute to per second because the unit system is
        // weird when converting rotations to linear distance
        elevatorLinearVelocity.mut_replace(encoder.getVelocity() / 60.0,
                FeetPerSecond);
        return elevatorLinearVelocity.mut_times(ElevatorConstants.CONVERSION_FACTOR.in(Feet));
    }

    /**
     * @return Gets the elevator's height from the ground. Zero is the ground and
     *         positive is up.
     */
    private Distance getElevatorHeight() {
        elevatorHeight.mut_replace(ElevatorConstants.CONVERSION_FACTOR);
        elevatorHeight.mut_times(encoder.getPosition());
        elevatorHeight.mut_plus(ElevatorConstants.FLOOR_OFFSET);
        return elevatorHeight;
    }

    /**
     * @return Whether the PID and profile have reached the target height.
     */
    private boolean isAtPosition() {
        return pid.atGoal();
    }

    /**
     * Moves the elevator to a specific height. Will wait until the elevator is at
     * the requested height before moving on with a 0.25s debounce. Useful for
     * command chaining.
     * 
     * @param height The requested height from the ground. Zero is the ground and
     *               positive is up.
     * @return The command to move the elevator.
     */
    public Command moveElevatorCommand(Distance height) {
        return this
                .runOnce(() -> this.setTargetHeight(height))
                .andThen(Commands.waitUntil(this::isAtPosition))
                .andThen(Commands.waitSeconds(0.1));
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
        // feedforward.setKg(Preferences.getDouble("Elevator/Kg", 0));
        // feedforward.setKv(Preferences.getDouble("Elevator/Kv", 0));

        SmartDashboard.putBoolean("Elevator/At Position", isAtPosition());
        SmartDashboard.putNumber("Elevator/Height (in)", getElevatorHeight().in(Inches));
    }
}
