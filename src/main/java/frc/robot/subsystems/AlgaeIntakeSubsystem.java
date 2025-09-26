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
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig;
    private final Trigger algaeDetected;

    public AlgaeIntakeSubsystem() {
        motor = new SparkMax(AlgaeIntakeConstants.MOTOR_ID, MotorType.kBrushed);
        motorConfig = new SparkMaxConfig();

        // configure sparkmax
        motorConfig
                .smartCurrentLimit(AlgaeIntakeConstants.CURRENT_LIMIT)
                .inverted(AlgaeIntakeConstants.INVERTED)
                .idleMode(AlgaeIntakeConstants.IDLE_MODE);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // When we detect that a algae has intaked, stop intaking and start holding the
        // algae to add grip while the robot moves. This runs until interrupted from
        // ejecting the algae.
        algaeDetected = new Trigger(() -> motor.getOutputCurrent() >= (AlgaeIntakeConstants.CURRENT_LIMIT - 1))
                .debounce(AlgaeIntakeConstants.CURRENT_DEBOUNCE_S)
                .onTrue(holdAlgaeCommand());

        // When the intake commands (in, out) get interrupted by either the button
        // being released or the command timer expiring, stop the motor and then do
        // nothing
        setDefaultCommand(this.runOnce(() -> runIntake(0.0)).andThen(this.run(() -> {
            /* idle */})).withName("Idle"));
    }

    /**
     * @return Gets whether algae has been detected as intaked.
     */
    public Trigger getAlgaeDetected() {
        return algaeDetected;
    }

    /**
     * Runs the intake at a particular speed.
     * 
     * @param speed a percentage from [-1, 1].
     */
    private void runIntake(double speed) {
        // setting voltage is more repeatable, so we convert from percentage to voltage
        // assuming a max voltage of 12V
        motor.setVoltage(speed * 12.0);
    }

    /**
     * Intakes algae. A successful intake will interrupt this command, otherwise
     * this should be stopped by the calling source (i.e.
     * {@link Trigger#whileTrue(Command)} to make the command stop when the trigger
     * has stopped). Runs indefinitely until interrupted.
     * 
     * @return command to intake algae
     */
    public Command intakeAlgaeCommand() {
        return this.run(() -> this.runIntake(AlgaeIntakeConstants.INTAKE_SPEED)).withName("IntakingAlgae");
    }

    /**
     * Command to eject algae. Runs for 0.5 seconds and then ends.
     * 
     * @return command to eject algae
     */
    public Command ejectAlgaeCommand() {
        return this.run(() -> this.runIntake(-AlgaeIntakeConstants.EJECT_SPEED))
                .withTimeout(AlgaeIntakeConstants.EJECT_TIME)
                .withName("EjectingAlgae");
    }

    /**
     * Runs the motors backwards at a slow speed to help grip the algae. Runs
     * indefinitely until interrupted.
     * 
     * @return command to hold algae
     */
    private Command holdAlgaeCommand() {
        return this.run(() -> this.runIntake(-AlgaeIntakeConstants.HOLD_SPEED)).withName("HoldingAlgae");
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("AlgaeIntake/Detected", algaeDetected.getAsBoolean());
        SmartDashboard.putNumber("AlgaeIntake/Intake Current (A)", motor.getOutputCurrent());
    }
}
