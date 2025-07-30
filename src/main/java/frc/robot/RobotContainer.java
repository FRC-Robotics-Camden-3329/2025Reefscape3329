package frc.robot;

import frc.robot.Constants.ReefConstants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDriveToReefCommand;
import frc.robot.subsystems.*;
import frc.robot.util.CalibrateQuestCommand;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
	private Algae m_Algae = new Algae();
	// private Climb m_Climb = new Climb();
	public final CommandXboxController m_driverController = new CommandXboxController(
			OperatorConstants.kDriverControllerPort);
	public final CommandXboxController m_operatorController = new CommandXboxController(
			OperatorConstants.kOperatorControllerPort);
	private CalibrateQuestCommand calibrateQuestCommand;

	private boolean autoDriving = false;

	public RobotContainer() {
		calibrateQuestCommand = new CalibrateQuestCommand();
		NamedCommands.registerCommand("L2Config", moveTo(ReefConstants.Level.L2));
		NamedCommands.registerCommand("L3Config", moveTo(ReefConstants.Level.L3));
		NamedCommands.registerCommand("L4Config", moveTo(ReefConstants.Level.L4));
		NamedCommands.registerCommand("A1Config", moveTo(ReefConstants.Level.A1));
		NamedCommands.registerCommand("CSConfig", moveTo(ReefConstants.Level.CS));
		NamedCommands.registerCommand("IntakeCoral", m_Coral.intakeCoralCommand());
		NamedCommands.registerCommand("EjectCoral", m_Coral.ejectCoralCommand());
		NamedCommands.registerCommand("IntakeAlgae", m_Coral.intakeCoralCommand());
		NamedCommands.registerCommand("EjectAlgae", m_Coral.ejectCoralCommand());

		setMotorBrake(false);
		drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
		autoChooser.setDefaultOption("None", null);
		configureBindings();

		m_Coral.getCoralDetected().onTrue(rumbleControllers(0.8, 0.5));
		m_Algae.getAlgaeDetected().onTrue(rumbleControllers(0.8, 0.5));
	}

	/**
	 * Rumbles both the driver and operator controllers for a duration.
	 * 
	 * @param strength        rumble strength from 0 to 1
	 * @param durationSeconds how many seconds to rumble the controllers for
	 * @return the command to rumble the controllers
	 */
	private Command rumbleControllers(double strength, double durationSeconds) {
		return Commands
				.run(() -> {
					m_driverController.setRumble(RumbleType.kBothRumble, strength);
					m_operatorController.setRumble(RumbleType.kBothRumble, strength);
				})
				.withTimeout(durationSeconds)
				.andThen(() -> {
					m_driverController.setRumble(RumbleType.kBothRumble, 0);
					m_operatorController.setRumble(RumbleType.kBothRumble, 0);
				});
	}

	SwerveInputStream driveAngularVelocity = SwerveInputStream
			.of(drivebase.getSwerveDrive(), () -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX())
			.withControllerRotationAxis(() -> -m_driverController.getRightX()).deadband(OperatorConstants.DEADBAND)
			.scaleTranslation(0.8)
			.translationOnlyWhile(
					() -> !autoDriving && Math.abs(m_driverController.getRightX()) < OperatorConstants.DEADBAND)
			.allianceRelativeControl(true);

	Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

	private void configureBindings() {
		m_operatorController.povLeft().onTrue(moveTo(ReefConstants.Level.L1));
		m_operatorController.povDown().onTrue(moveTo(ReefConstants.Level.L2));
		m_operatorController.povUp().onTrue(moveTo(ReefConstants.Level.L3));
		m_operatorController.povRight().onTrue(moveTo(ReefConstants.Level.L4));

		m_operatorController.a().onTrue(moveTo(ReefConstants.Level.CS));
		m_operatorController.b().onTrue(moveTo(ReefConstants.Level.A1));
		m_operatorController.y().onTrue(moveTo(ReefConstants.Level.A2));
		m_operatorController.x().onTrue(moveTo(ReefConstants.Level.P));

		// intake coral
		m_operatorController.leftBumper()
				.whileTrue(m_Coral.intakeCoralCommand()
						.andThen(m_Coral.moveCoralCommand(CoralConstants.STOWED)));
		// eject coral
		m_operatorController.rightBumper()
				.onTrue(m_Coral.ejectCoralCommand()
						.andThen(m_Coral.moveCoralCommand(CoralConstants.STOWED)));
		m_operatorController.back().onTrue(m_Coral.ejectCoralSlowCommand());
		// intake algae
		m_operatorController.leftTrigger()
				.whileTrue(m_Algae.intakeAlgaeCommand()
						.andThen(m_Algae.moveAlgaeCommand(Degrees.of(50.0))));
		// eject algae
		m_operatorController.rightTrigger().onTrue(m_Algae.ejectAlgaeCommand());

		// m_driverController.leftBumper().whileTrue(drivebase.getAngleCharacterizationCommand());
		// m_driverController.rightBumper().whileTrue(drivebase.getDriveCharacterizationCommand());

		// m_driverController.a()
		// .whileTrue(
		// // toggleing tracking is required because the heading will be the same as it
		// was
		// // before path following if not toggled
		// Commands.runOnce(() -> tracking = true)
		// .andThen(drivebase.getPathFindingCommand()))
		// .onFalse(Commands.runOnce(() -> tracking = false));

		m_driverController.leftBumper()
				.whileTrue(autoDriving(new AutoDriveToReefCommand(ReefConstants.REEF_LEFT, drivebase)));
		m_driverController.y()
				.whileTrue(autoDriving(new AutoDriveToReefCommand(ReefConstants.REEF_CENTER, drivebase)));
		m_driverController.rightBumper()
				.whileTrue(autoDriving(new AutoDriveToReefCommand(ReefConstants.REEF_RIGHT, drivebase)));

		m_driverController.x()
				.onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3.2, 4.05, Rotation2d.kZero))));

		m_driverController.back().whileTrue(calibrateQuestCommand.determineOffsetToRobotCenter(drivebase, questNav));

		// m_driverController.povUp().whileTrue(m_Climb.climbCommand());
		// m_driverController.povDown().whileTrue(m_Climb.reachCommand());

		SmartDashboard.putData("Level 1", moveTo(ReefConstants.Level.L1));
		SmartDashboard.putData("Level 2", moveTo(ReefConstants.Level.L2));
		SmartDashboard.putData("Level 3", moveTo(ReefConstants.Level.L3));
		SmartDashboard.putData("Level 4", moveTo(ReefConstants.Level.L4));
	}

	/**
	 * Reconfigures the robot to the selected level/action. If the level is a coral
	 * level, the robot will eject the coral and then go back to the intaking
	 * configuration.
	 * 
	 * @param level the desired level to move the robot to
	 * @return the command that reconfigures the robot
	 */
	private Command moveTo(ReefConstants.Level level) {
		try {
			/**
			 * This is using some Java "magic" to get the class field corresponding to the
			 * level name. For example, this requires `ElevatorConstants` to have the field
			 * `L1` because `ReefConstants.L1` exists. It then gets the `L1` value from
			 * `ElevatorConstants`. I googled "java get class variable by string name" for
			 * this. It requires the try/catch block in case the field does not exist.
			 * Probably not the best way to do this...
			 */
			Distance elevatorHeight = (Distance) ElevatorConstants.class.getDeclaredField(level.name()).get(null);
			Angle algaeAngle = (Angle) AlgaeConstants.class.getDeclaredField(level.name()).get(null);
			Angle coralAngle = (Angle) CoralConstants.class.getDeclaredField(level.name()).get(null);

			// handle the coral seperately to wait for moving the coral and score once it
			// gets to that position
			if (level == ReefConstants.Level.L1 || level == ReefConstants.Level.L2
					|| level == ReefConstants.Level.L3 || level == ReefConstants.Level.L4) {
				return Commands
						.parallel(
								m_Elevator.moveElevatorCommand(elevatorHeight),
								m_Algae.moveAlgaeCommand(algaeAngle))
						.andThen(m_Coral.moveCoralCommand(coralAngle))
						.andThen(m_Coral.ejectCoralCommand())
						.andThen(moveTo(ReefConstants.Level.CS));
			} else {
				// all the other positions just move all three subsystems at the same time
				return Commands.parallel(
						m_Elevator.moveElevatorCommand(elevatorHeight),
						m_Algae.moveAlgaeCommand(algaeAngle),
						m_Coral.moveCoralCommand(coralAngle))
						.andThen(rumbleControllers(0.2, 0.2));
			}
		} catch (Exception e) {
			// if each configuration is tested, this should not ever be reached
			DriverStation.reportError("Cannot decode level!! (do all subsystems have all the congurations?)",
					true);
			return Commands.none();
		}
	}

	/**
	 * Method to wrap any auto driving command. This is required because it
	 * prevents the drivetrain from snapping back to the angle it was at before
	 * starting the auto driving command. This is required due to how
	 * {@code translationOnlyWhile() } works and how the controller processor
	 * updates the heading lock.
	 * 
	 * @param drivingCommand The command to auto drive.
	 * @return The wrapped command to auto drive.
	 */
	private Command autoDriving(Command drivingCommand) {
		return drivingCommand.beforeStarting(() -> {
			// set internal state to tracking
			autoDriving = true;
			// update state in the controller supplier (required)
			driveAngularVelocity.get();
		}).finallyDo(interrupted -> {
			autoDriving = false;
			driveAngularVelocity.get();
		});
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
