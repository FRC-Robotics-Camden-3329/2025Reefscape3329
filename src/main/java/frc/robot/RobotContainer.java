package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Configs.*;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  private QuestNavSubsystem questNav = new QuestNavSubsystem();
  public final SwerveSubsystem drivebase = new SwerveSubsystem(questNav::setQuestPose, questNav::getVisionData);
  private Elevator m_Elevator = new Elevator();
  private Coral m_Coral = new Coral();
  private Algae m_Algae = new Algae(m_Elevator);
  // private Climb m_Climb = new Climb();
  public final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  public final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);
  private boolean tracking = false;

  public RobotContainer() {
    NamedCommands.registerCommand("L2Config", new L2Config(m_Elevator, m_Coral, m_Algae));
    NamedCommands.registerCommand("L3Config", new L3Config(m_Elevator, m_Coral, m_Algae));
    NamedCommands.registerCommand("L4Config", new L4Config(m_Elevator, m_Coral, m_Algae));
    NamedCommands.registerCommand("A1Config", new A1Config(m_Elevator, m_Coral, m_Algae));
    NamedCommands.registerCommand("CSConfig", new CSConfig(m_Elevator, m_Coral, m_Algae));
    NamedCommands.registerCommand("IntakeCoral", m_Coral.runIntakeCommand(Constants.CoralConstants.intakeSpeed));
    NamedCommands.registerCommand("EjectCoral", m_Coral.runIntakeCommand(-Constants.CoralConstants.ejectSpeed));
    NamedCommands.registerCommand("StopCoral", m_Coral.runIntakeCommand(0));
    NamedCommands.registerCommand("IntakeAlgae",
        m_Coral.runOnce(() -> m_Algae.runIntake(Constants.AlgaeConstants.intakeSpeed)));
    NamedCommands.registerCommand("EjectAlgae",
        m_Coral.runOnce(() -> m_Algae.runIntake(Constants.AlgaeConstants.ejectSpeed)));
    NamedCommands.registerCommand("StopAlgae", m_Coral.runOnce(() -> m_Algae.runIntake(0)));

    setMotorBrake(true);
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoChooser.setDefaultOption("None", null);
    configureBindings();
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream
      .of(drivebase.getSwerveDrive(), () -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> -m_driverController.getRightX()).deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .translationOnlyWhile(() -> !tracking && Math.abs(m_driverController.getRightX()) < OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  private void configureBindings() {
    m_operatorController.povLeft().onTrue(new L1Config(m_Elevator, m_Coral, m_Algae));
    m_operatorController.povDown().onTrue(new L2Config(m_Elevator, m_Coral, m_Algae));
    m_operatorController.povUp().onTrue(new L3Config(m_Elevator, m_Coral, m_Algae));
    m_operatorController.povRight().onTrue(new L4Config(m_Elevator, m_Coral, m_Algae));
    m_operatorController.a().onTrue(new CSConfig(m_Elevator, m_Coral, m_Algae));
    m_operatorController.leftBumper().whileTrue(m_Coral.intakeCoralCommand());
    m_driverController.rightTrigger().whileTrue(m_Coral.ejectCoralCommand());

    m_operatorController.b().onTrue(new A1Config(m_Elevator, m_Coral, m_Algae));
    m_operatorController.y().onTrue(new A2Config(m_Elevator, m_Coral, m_Algae));
    m_operatorController.x().onTrue(new PConfig(m_Elevator, m_Coral, m_Algae));
    m_operatorController.leftTrigger().whileTrue(m_Algae.intakeAlgaeCommand());
    m_operatorController.rightTrigger().whileTrue(m_Algae.ejectAlgaeCommand());

    m_driverController.a()
        .whileTrue(
            // toggleing tracking is required because the heading will be the same as it was
            // before path following if not toggled
            Commands.runOnce(() -> tracking = true)
                .andThen(drivebase.getPathFindingCommand()))
        .onFalse(Commands.runOnce(() -> tracking = false));

    m_driverController.x().onTrue(drivebase.zeroGyro());

    // m_driverController.povUp().whileTrue(m_Climb.climbCommand());
    // m_driverController.povDown().whileTrue(m_Climb.reachCommand());

    SmartDashboard.putData("Level 1", new L1Config(m_Elevator, m_Coral, m_Algae));
    SmartDashboard.putData("Level 2", new L2Config(m_Elevator, m_Coral, m_Algae));
    SmartDashboard.putData("Level 3", new L3Config(m_Elevator, m_Coral, m_Algae));
    SmartDashboard.putData("Level 4", new L4Config(m_Elevator, m_Coral, m_Algae));
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
