package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntakeSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final SparkMaxConfig intakeConfig;
    private final Trigger coralDetected;

    public CoralIntakeSubsystem() {
        motor = new SparkMax(CoralIntakeConstants.MOTOR_ID, MotorType.kBrushless);
        intakeConfig = new SparkMaxConfig();

        // configure sparkmax
        intakeConfig
                .smartCurrentLimit(CoralIntakeConstants.CURRENT_LIMIT)
                .inverted(CoralIntakeConstants.INVERTED)
                .idleMode(CoralIntakeConstants.IDLE_MODE);
        motor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // When we detect that a coral has intaked, stop intaking and start holding the
        // coral to add grip while the robot moves. This runs until interrupted from
        // ejecting the coral
        coralDetected = new Trigger(() -> motor.getOutputCurrent() >= (CoralIntakeConstants.CURRENT_LIMIT - 1))
                .debounce(CoralIntakeConstants.CURRENT_DEBOUNCE_S)
                .onTrue(holdCoralCommand());

        // When the intake commands (in, out) get interrupted by either the button
        // being released or the command timer expiring, stop the motor and then do
        // nothing
        setDefaultCommand(this.runOnce(() -> runIntake(0.0)).andThen(this.run(() -> {
            /* idle */})).withName("Idle"));
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
     * Runs the intake motor.
     * 
     * @param speed set speed from [-1,1]
     */
    private void runIntake(double speed) {
        // setting voltage is more repeatable, so we convert from percentage to voltage
        // assuming a max voltage of 12V
        motor.setVoltage(speed * 12.0);
    }

    /**
     * Runs the intake. Will run the intake motors backwards to hold onto the coral
     * if the game piece is coral was detected. Runs indefinitely until interrupted.
     * 
     * @return run intake command
     */
    public Command intakeCoralCommand() {
        return this.run(() -> runIntake(CoralIntakeConstants.INTAKE_SPEED)).withName("IntakingCoral");
    }

    /**
     * Ejects the coral. Command ends after 0.5 seconds.
     * 
     * @return eject coral command
     */
    public Command ejectCoralCommand() {
        return this.run(() -> runIntake(-CoralIntakeConstants.EJECT_SPEED)).withTimeout(CoralIntakeConstants.EJECT_TIME)
                .withName("EjectingCoral");
    }

    /**
     * Ejects the coral slower. Command ends after 0.5 seconds.
     * 
     * @return eject coral command
     */
    public Command ejectCoralSlowCommand() {
        return this.run(() -> runIntake(-CoralIntakeConstants.EJECT_SPEED / 2.0))
                .withTimeout(CoralIntakeConstants.EJECT_TIME)
                .withName("EjectingCoralSlowly");
    }

    /**
     * Runs the intake motor at a slow speed inwards to hold onto the coral. Runs
     * indefinitely until interrupted.
     * 
     * @return the command to hold onto the coral
     */
    private Command holdCoralCommand() {
        return this.run(() -> runIntake(CoralIntakeConstants.HOLD_SPEED)).withName("HoldingCoral");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("CoralIntake/Intake Current (A)", motor.getOutputCurrent());
        SmartDashboard.putBoolean("CoralIntake/Detected", getCoralDetected().getAsBoolean());
    }
}
