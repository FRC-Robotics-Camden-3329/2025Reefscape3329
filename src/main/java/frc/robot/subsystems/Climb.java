package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private SparkMax motor;
    private SparkMaxConfig config;

    public Climb() {
        motor = new SparkMax(Constants.ClimbConstants.motorID, SparkMax.MotorType.kBrushed);
        config = new SparkMaxConfig();

        config
                .smartCurrentLimit(70)
                .idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void run(double speed) {
        motor.set(speed);
    }

    public Command climbCommand() {
        return this.runEnd(() -> this.run(Constants.CoralIntakeConstants.INTAKE_SPEED), () -> this.run(0));
    }

    public Command reachCommand() {
        return this.runEnd(() -> this.run(-Constants.CoralIntakeConstants.INTAKE_SPEED), () -> this.run(0));
    }
}
