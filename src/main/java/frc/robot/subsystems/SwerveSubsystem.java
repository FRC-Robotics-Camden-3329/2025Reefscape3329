package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ReefConstants;
import frc.robot.util.VisionData;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

public class SwerveSubsystem extends SubsystemBase {
	private File directory = new File(Filesystem.getDeployDirectory(), "swerve");
	private SwerveDrive swerveDrive;
	private Command pathfindingCommand;

	/**
	 * Creates a new swerve subsystem.
	 * 
	 * @param resetVision        A consumer that accepts a {@link Pose2d} to reset
	 *                           the vision system's position when requested
	 * @param visionDataSupplier A supplier for giving {@link VisionData} to the
	 *                           drivebase.
	 */
	public SwerveSubsystem() {
		// Optionally allow for a set maxSpeed to be defined in smartDashboard, default to Consants.maxSpeed
        double maxSpeed = SmartDashboard.getNumber("Drive speed (ft)", Constants.maxSpeed);
        SmartDashboard.putNumber("Drive speed (ft)", maxSpeed);

		// Initalize swerve drives
		try {
			swerveDrive = new SwerveParser(directory).createSwerveDrive(maxSpeed,
					new Pose2d(new Translation2d(Meter.of(0), Meter.of(0)), Rotation2d.fromDegrees(0)));
		} catch (Exception e) {
			throw new RuntimeException(e);
		}

		swerveDrive.setChassisDiscretization(true, 0.02);
		swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(0.0846525, 2.68855, 0.2266775));

		setupPathPlanner();

		// Auto pathing
		// Load the path we want to pathfind to and follow
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile("Auto Path");
			// Create the constraints to use while pathfinding. The constraints defined in
			// the path will only be used for the path.
			PathConstraints constraints = new PathConstraints(
					2.0, 1.0,
					Units.degreesToRadians(720), Units.degreesToRadians(360));

			// Since AutoBuilder is configured, we can use it to build pathfinding commands
			pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
					path,
					constraints);
		} catch (Exception e) {
			DriverStation.reportError("Unable to load autonomus path for path following!!", e.getStackTrace());
			pathfindingCommand = Commands.none();
		}
	}

	/**
	 * Gets the robot's nearest side for the coral station. Will take into account
	 * the current alliance. Used in {@code getTransformedCS}.
	 * 
	 * @return the nearest side of the coral station.
	 */
	public ReefConstants.Side getNearestSide() {
		// https://www.desmos.com/calculator/es1kjb24zg
		Translation2d currTranslation = swerveDrive.getPose().getTranslation();
		Translation2d reefPositionWS = ReefConstants.reefPositionWS;
		var alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			reefPositionWS = FlippingUtil.flipFieldPosition(reefPositionWS);
		}

		// Convert from worldspace to reefspace
		currTranslation = currTranslation.minus(reefPositionWS);
		currTranslation = currTranslation.rotateBy(Rotation2d.fromRadians(-0.5));

		int segment = (int) (Math
				.floor(Math.atan2(currTranslation.getY(), currTranslation.getX())
						/ 1.04719755 /* aka 60 degrees in radians */));
		switch (segment) {
			case 2:
				return ReefConstants.Side.AB;
			case 1:
				return ReefConstants.Side.KL;
			case 0:
				return ReefConstants.Side.IJ;
			case -1:
				return ReefConstants.Side.GH;
			case -2:
				return ReefConstants.Side.EF;
			case -3:
				return ReefConstants.Side.CD;
			default:
				// return AB on invalid
				// we should never hit this because of how atan2 works, but avoid returning null
				// to prevent program from crashing
				return ReefConstants.Side.AB;
		}
	}

	/**
	 * Transforms either the left, right, or middle reef station positions to the
	 * correct world space position.
	 * 
	 * @param pose the pose to be transformed by. This assumes the pose will be
	 *             from the AB side in world space coordinates on the blue side.
	 * @param side the side the final transformation should be on
	 * @return the correct world space position of the nearest coral station
	 */
	public Pose2d getReefTransformation(Pose2d pose, ReefConstants.Side side) {
		Rotation2d rot = Rotation2d.fromDegrees(60.0).times(side.ordinal());
		Optional<Alliance> alliance = DriverStation.getAlliance();

		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			return FlippingUtil.flipFieldPose(pose)
					.rotateAround(
							FlippingUtil.flipFieldPosition(ReefConstants.reefPositionWS),
							rot.rotateBy(Rotation2d.k180deg));
		} else {
			return pose.rotateAround(ReefConstants.reefPositionWS, rot);
		}
	}

	/**
	 * Gets the path following command. Probably will need to replace this with
	 * something more robust when adding more than one path.
	 * 
	 * @return The command to run this path.
	 */
	public Command getPathFindingCommand() {
		return pathfindingCommand.finallyDo(() -> swerveDrive.drive(new ChassisSpeeds(0, 0, 0)));
	}

	@Override
	public void periodic() {
		SmartDashboard.putString("Swerve/Nearest Side", getNearestSide().toString());
		swerveDrive.field.getObject("GoalSide")
				.setPose(getReefTransformation(ReefConstants.REEF_CENTER, getNearestSide()));
	}

	@Override
	public void simulationPeriodic() {
	}

	/**
	 * Gets the YAGSL {@link SwerveDrive} in this subsystem.
	 * 
	 * @return this subsystem's {@link SwerveDrive}
	 */
	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	/**
	 * See
	 * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
	 */
	public void addVisionMeasurement(
			Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
		swerveDrive.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
	}

	/**
	 * Drives the swervedrive field oriented.
	 * 
	 * @param velocity the field oriented {@link ChassisSpeeds}
	 */
	public void driveFieldOriented(ChassisSpeeds velocity) {
		swerveDrive.driveFieldOriented(velocity);
	}

	/**
	 * Command to drive the robot field oriented.
	 * 
	 * @param velocity a {@link ChassisSpeeds} {@link Supplier} to provide data
	 * @return
	 */
	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
		return run(() -> {
			swerveDrive.driveFieldOriented(velocity.get());
		});
	}

	/**
	 * Another drive command
	 * 
	 * @param translationX
	 * @param translationY
	 * @param headingX
	 * @param headingY
	 * @return
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
			DoubleSupplier headingY) {
		return run(() -> {
			Translation2d scaledInputs = SwerveMath.scaleTranslation(
					new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);
			driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
					headingX.getAsDouble(), headingY.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(),
					swerveDrive.getMaximumChassisVelocity()));
		});
	}

	/**
	 * Whether the drivetrain should be in brake or coast mode.
	 * 
	 * @param brake {@code true} is brake, {@code false} is coast
	 */
	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	/**
	 * Command to zero the gyro. Will wait 0.5s after zeroing.
	 * 
	 * @return command to zero the gyro
	 */
	public Command zeroGyro() {
		return Commands.runOnce(() -> swerveDrive.zeroGyro()).andThen(Commands.waitSeconds(0.5));
	}

	/**
	 * Gets the gyro's yaw.
	 * 
	 * @return the gyro's yaw
	 */
	public Rotation2d getGyro() {
		return swerveDrive.getYaw();
	}

	/**
	 * Reset's both the vision and encoder/gyro odometry positions.
	 * <p>
	 * <b>note: don't set it to (0, 0).</b> for some reason the Quest does not reset
	 * the position when commanded to (0, 0).
	 * 
	 * @param pose the worldspace positon to set the robot.
	 */
	public void resetOdometry(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}

	/**
	 * @return angle motor characterization command
	 */
	public Command getAngleCharacterizationCommand() {
		return SwerveDriveTest.generateSysIdCommand(
				SwerveDriveTest.setAngleSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive),
				3,
				6,
				3);
	}

	/**
	 * @return drive motor characterization command
	 */
	public Command getDriveCharacterizationCommand() {
		return SwerveDriveTest.generateSysIdCommand(
				SwerveDriveTest.setDriveSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive, 12, true),
				3,
				4,
				1.5);
	}

	public void setupPathPlanner() {
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
			final boolean enableFeedforward = true;
			AutoBuilder.configure(swerveDrive::getPose, swerveDrive::resetOdometry, swerveDrive::getRobotVelocity,
					(speedsRobotRelative, moduleFeedForwards) -> {
						if (enableFeedforward) {
							swerveDrive.drive(speedsRobotRelative,
									swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
									moduleFeedForwards.linearForces());
						} else {
							swerveDrive.setChassisSpeeds(speedsRobotRelative);
						}
					}, new PPHolonomicDriveController(new PIDConstants(3.0, 0.0, 0.1), new PIDConstants(3.0, 0.0, 0.1)),
					config,
					() -> {
						var alliance = DriverStation.getAlliance();
						if (alliance.isPresent()) {
							return alliance.get() == DriverStation.Alliance.Red;
						}
						return false;
					}, this);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Gets the path by name.
	 * 
	 * @param pathName name of the path
	 * @return the command to follow the path
	 */
	public Command getAutonomousCommand(String pathName) {
		return new PathPlannerAuto(pathName);
	}
}
