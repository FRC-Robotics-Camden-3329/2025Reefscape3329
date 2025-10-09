package frc.robot;

import frc.robot.Constants.ReefConstants;
import frc.robot.Constants.SubsystemStates;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDriveToReefCommand;
import frc.robot.commands.AutoDriveToGamePieceCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.CalibrateQuest2Command;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
	private final SendableChooser<Command> autoChooser;

	private final SwerveSubsystem drivebase = new SwerveSubsystem();
	private final QuestNavSubsystem questNav = new QuestNavSubsystem(drivebase::addVisionMeasurement, true);
	private final PhotonVisionSubsystem photonVision = new PhotonVisionSubsystem(
			drivebase::addVisionMeasurement, false);
	private final GamePieceDetector gamePieceDetector = new GamePieceDetector(
			() -> drivebase.getSwerveDrive().getPose(), drivebase.getSwerveDrive().field);

	private final Elevator elevator = new Elevator();
	private final CoralPivotSubsystem coralPivot = new CoralPivotSubsystem();
	private final AlgaePivotSubsystem algaePivot = new AlgaePivotSubsystem();
	private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();
	private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
	// private Climb m_Climb = new Climb();

	public final CommandXboxController m_driverController = new CommandXboxController(
			OperatorConstants.kDriverControllerPort);
	public final CommandXboxController m_operatorController = new CommandXboxController(
			OperatorConstants.kOperatorControllerPort);

	private boolean autoDriving = false;

	public RobotContainer() {
		resetOdometry(new Pose2d(1, 1, Rotation2d.kZero));

		// On teleop, reset the odometry to the PV result only if not connected to FMS.
		// At comp, we only want to reset the pose at the start of auton because we know
		// the robot will be looking at a tag. We cannot garentee that at the start of
		// teleop.
		new Trigger(RobotState::isTeleop).and(new Trigger(DriverStation::isFMSAttached).negate())
				.onTrue(autoDriving(
						Commands.runOnce(() -> photonVision.getPoseOptional().ifPresent(pose -> resetOdometry(pose)))));

		/*
		 * In the event that QN stops tracking, failover to PV for updating our global
		 * robot pose. This will be accurate but require a tag in sight so it may
		 * become inaccurate if the robot cannot see any tags.
		 */
		new Trigger(questNav::isTracking).debounce(0.5, DebounceType.kFalling)
				.onFalse(Commands.runOnce(() -> {
					DriverStation.reportError(
							"QuestNav tracking lost!! Failing over to PhotonVision...",
							false);
					questNav.setUseEstConsumer(false);
					photonVision.setUseEsimationConsumer(true);
				}));

		// Configure commands for PathPlanner.
		// Autons created in PathPlanner must not reset the position of the robot
		// because PV will update the robot's world position.
		NamedCommands.registerCommand("L2Config", changeSubsystemStates(SubsystemStates.L2));
		NamedCommands.registerCommand("L3Config", changeSubsystemStates(SubsystemStates.L3));
		NamedCommands.registerCommand("L4Config", changeSubsystemStates(SubsystemStates.L4));
		NamedCommands.registerCommand("A1Config", changeSubsystemStates(SubsystemStates.A1));
		NamedCommands.registerCommand("CSConfig", changeSubsystemStates(SubsystemStates.CS));
		NamedCommands.registerCommand("IntakeCoral", coralIntake.intakeCoralCommand());
		NamedCommands.registerCommand("EjectCoral", coralIntake.ejectCoralCommand());
		NamedCommands.registerCommand("IntakeAlgae", algaeIntake.intakeAlgaeCommand());
		NamedCommands.registerCommand("EjectAlgae", algaeIntake.ejectAlgaeCommand());

		setMotorBrake(false);
		drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
		// do nothing at home, switch to just moving off the line when at comp
		autoChooser.setDefaultOption("None", null);
		configureBindings();

		// Rumble the controllers whenever we pickup a game piece
		coralIntake.getCoralDetected().onTrue(rumbleControllers(0.8, 0.5));
		algaeIntake.getAlgaeDetected().onTrue(rumbleControllers(0.8, 0.5));
	}

	/**
	 * Resets both the drivetrain's position and sets the quest's position.
	 * 
	 * @param pose the position in the world
	 */
	private void resetOdometry(Pose2d pose) {
		questNav.setQuestPose(pose);
		drivebase.resetOdometry(pose);
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
		m_operatorController.povLeft().onTrue(changeSubsystemStates(SubsystemStates.L1));
		m_operatorController.povDown().onTrue(changeSubsystemStates(SubsystemStates.L2));
		m_operatorController.povUp().onTrue(changeSubsystemStates(SubsystemStates.L3));
		m_operatorController.povRight().onTrue(changeSubsystemStates(SubsystemStates.L4));

		m_operatorController.a().onTrue(changeSubsystemStates(SubsystemStates.CS));
		m_operatorController.b().onTrue(changeSubsystemStates(SubsystemStates.A1));
		m_operatorController.y().onTrue(changeSubsystemStates(SubsystemStates.A2));
		m_operatorController.x().onTrue(changeSubsystemStates(SubsystemStates.P));

		// intake coral
		m_operatorController.leftBumper()
				.whileTrue(coralIntake.intakeCoralCommand()
						.andThen(coralPivot.moveCoralCommand(CoralPivotConstants.STOWED)));
		// eject coral
		m_operatorController.rightBumper()
				.onTrue(coralIntake.ejectCoralCommand()
						.andThen(coralPivot.moveCoralCommand(CoralPivotConstants.STOWED)));
		m_operatorController.back().onTrue(coralIntake.ejectCoralSlowCommand());
		// intake algae
		m_operatorController.leftTrigger()
				.whileTrue(algaeIntake.intakeAlgaeCommand()
						.andThen(algaePivot.moveAlgaeCommand(Degrees.of(50.0))));
		// eject algae
		m_operatorController.rightTrigger().onTrue(algaeIntake.ejectAlgaeCommand());

		// m_driverController.leftBumper().whileTrue(drivebase.getAngleCharacterizationCommand());
		// m_driverController.rightBumper().whileTrue(drivebase.getDriveCharacterizationCommand());

		// m_driverController.a()
		// .whileTrue(
		// autoDriving(drivebase.getPathFindingCommand()));

		m_driverController.b().whileTrue(autoDriving(new CalibrateQuest2Command(drivebase, questNav)));

		m_driverController.leftBumper()
				.whileTrue(autoDriving(new AutoDriveToReefCommand(ReefConstants.REEF_LEFT, drivebase)));
		m_driverController.y()
				.whileTrue(autoDriving(new AutoDriveToReefCommand(ReefConstants.REEF_CENTER, drivebase)));
		m_driverController.rightBumper()
				.whileTrue(autoDriving(new AutoDriveToReefCommand(ReefConstants.REEF_RIGHT, drivebase)));

		m_driverController.x()
				.onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3.2, 4.05, Rotation2d.kZero))));

		m_driverController.a().whileTrue(
				autoDriving(new AutoDriveToGamePieceCommand(gamePieceDetector::getNearestGamePiecePose, drivebase)));

		// m_driverController.povUp().whileTrue(m_Climb.climbCommand());
		// m_driverController.povDown().whileTrue(m_Climb.reachCommand());

		SmartDashboard.putData("Level 1", changeSubsystemStates(SubsystemStates.L1));
		SmartDashboard.putData("Level 2", changeSubsystemStates(SubsystemStates.L2));
		SmartDashboard.putData("Level 3", changeSubsystemStates(SubsystemStates.L3));
		SmartDashboard.putData("Level 4", changeSubsystemStates(SubsystemStates.L4));

		// Add button to the dashboard to reset QN's position with PV
		SmartDashboard.putData("Reset Quest Position From PhotonVision", Commands.runOnce(() -> {
			DataLogManager.log("Attempting to reset QN to PV pose...");

			photonVision.getPoseOptional().ifPresent(pose -> {
				photonVision.setUseEsimationConsumer(false);
				questNav.setQuestPose(pose);
				questNav.setUseEstConsumer(true);
				DataLogManager.log("QN pose successfully set from PV!");
			});
		}));
	}

	/**
	 * Reconfigures the robot to the desired subsystem states. If the state is a
	 * coral state, the robot will eject the coral and then go back to the intaking
	 * state.
	 * 
	 * @param state the desired state to move the robot to
	 * @return the command that reconfigures the robot, ends once the state has been
	 *         reached
	 */
	private Command changeSubsystemStates(SubsystemStates state) {
		// Handle the coral seperately to wait until the elevator has reached its state
		// to pivot and eject. This prevents the coral from hitting the reef on the way
		// up.
		if (state.isCoral()) {
			return Commands.sequence(
					Commands.parallel(
							elevator.moveElevatorCommand(state.elevatorHeight()),
							algaePivot.moveAlgaeCommand(state.algaeAngle())),
					coralPivot.moveCoralCommand(state.coralAngle()),
					coralIntake.ejectCoralCommand(),
					changeSubsystemStates(SubsystemStates.CS));
		} else {
			// all the other positions just move all three subsystems at the same time
			return Commands
					.parallel(
							elevator.moveElevatorCommand(state.elevatorHeight()),
							algaePivot.moveAlgaeCommand(state.algaeAngle()),
							coralPivot.moveCoralCommand(state.coralAngle()))
					.andThen(rumbleControllers(0.2, 0.2));
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

	/**
	 * Gets the autonomous command according to the PathPlanner autonomous selector.
	 * Will reset the robot's position according to PhotonVision if PV has a valid
	 * position. Will not move if no autonomous is selected.
	 * 
	 * @return the autonomous command
	 */
	public Command getAutonomousCommand() {
		Command auton = autoChooser.getSelected();
		Optional<Pose2d> pvPose = photonVision.getPoseOptional();

		if (auton != null && pvPose.isPresent()) {
			// auton selected and valid pv pose

			return autoDriving(auton.beforeStarting(() -> resetOdometry(pvPose.get())));
		} else if (auton != null && pvPose.isEmpty()) {
			// auton selected and invalid pv pose
			DriverStation.reportError("Auton Error: Auton SELECTED but the PV POSE is INVALID!!", false);

			// can maybe change this to slowly move off the line, but need to be careful
			// because the robot doesn't know where it is and we don't want to get in the
			// way of our other alliance members.
			return Commands.none();
		} else if (auton == null && pvPose.isPresent()) {
			// no auton selected and valid pv pose
			DriverStation.reportError("Auton Error: Auton NOT SELECTED (PV POSE is VALID)!!", false);

			// can also maybe change this to just slowly move off the line in addition to
			// reseting the robot's pose
			return autoDriving(Commands.runOnce(() -> resetOdometry(pvPose.get())));
		} else {
			// no auton selected and invalid pv pose
			DriverStation.reportError("Auton Error: Auton NOT SELECTED and PV POSE is INVALID!!", false);

			// can also maybe change this to just slowly move off the line
			return Commands.none();
		}
	}
}
