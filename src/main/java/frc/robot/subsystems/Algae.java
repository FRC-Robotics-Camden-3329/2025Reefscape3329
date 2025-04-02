package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Algae extends SubsystemBase {
    ProfiledPIDController pid;
    SparkMax pivot, intake;
    DutyCycleEncoder encoder;
    double target;
    Elevator elevator;

    public Algae(Elevator elevator) {
        this.elevator = elevator;
        pid = new ProfiledPIDController(Constants.AlgaeConstants.kP,
                                        Constants.AlgaeConstants.kI,
                                        Constants.AlgaeConstants.kD,
                                        new Constraints(Constants.AlgaeConstants.maxVelocity,
                                                        Constants.AlgaeConstants.maxAcceleration));
        pivot = new SparkMax(Constants.AlgaeConstants.pivotID, MotorType.kBrushless);
        intake = new SparkMax(Constants.AlgaeConstants.intakeID, MotorType.kBrushed);
        encoder = new DutyCycleEncoder(Constants.AlgaeConstants.encoderID);
        setTarget(0.24);
    }

    public void runPivot(double speed) {
        pivot.set(-speed);
    }

    public void setTarget(double degrees) {
        target = degrees;
        pid.setGoal(target);
    }

    public double getEncoderPosition() {
        return encoder.get() - 0.499;
    }

    public boolean isAtPosition() {
        return pid.atGoal();
    }

    public Command moveAlgaeCommand(double degrees) {
        return Commands.runOnce(() -> this.setTarget(degrees))
                       .andThen(Commands.waitSeconds(0.5))
                       .until(() -> isAtPosition());
    }

    public void runIntake(double speed) {
        intake.set(-speed);
    }

    public Command intakeAlgaeCommand() {
        return this.runEnd(
            () -> this.runIntake(Constants.AlgaeConstants.intakeSpeed),
            () -> {
            this.runIntake(Constants.AlgaeConstants.holdSpeed);
            if (getEncoderPosition() > -0.05 && getEncoderPosition() < 0.05) {
                elevator.moveElevatorCommand(elevator.getEncoderPosition() + 1.4);
            }
            //moveAlgaeCommand(getEncoderPosition() + 0.05);
            }
        );
    }

    public Command ejectAlgaeCommand() {
        return this.runEnd(
            () -> this.runIntake(-Constants.AlgaeConstants.ejectSpeed),
            () -> this.runIntake(0)
        );
    }

    @Override
    public void periodic() {
        runPivot(pid.calculate(getEncoderPosition()));
        SmartDashboard.putNumber("Algae Position", getEncoderPosition());
        SmartDashboard.putNumber("Algae Target", target);
        SmartDashboard.putBoolean("Algae At Position", isAtPosition());
    }
}
