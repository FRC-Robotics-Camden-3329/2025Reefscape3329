package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public final class Constants {

  public final class ReefConstants {
    public static enum Side {
      AB, CD, EF, GH, IJ, KL
    };

    public static enum Level {
      L1, L2, L3, L4, A1, A2, CS, P
    };

    public static final Translation2d reefPositionWS = new Translation2d(4.489, 4.0259);
    public static final Pose2d REEF_LEFT = new Pose2d(3.292, 4.201, Rotation2d.kZero);
    public static final Pose2d REEF_CENTER = new Pose2d(3.292, 4.000, Rotation2d.kZero);
    public static final Pose2d REEF_RIGHT = new Pose2d(3.292, 3.840, Rotation2d.kZero);
  }

  public final class QuestConstants {
    public static final Transform2d ROBOT_TO_QUEST = new Transform2d(
        Meters.of(-0.222), // x, positive forward from robot center 8.74 in
        Meters.of(0.162), // y, positive left of robot center 5.944 in
        Rotation2d.k180deg); // rotation around the robot center
    public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );
  }

  public static class PVConstants {
    public static final String CAMERA_NAME = "Yellow Camera";
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        Inches.of(5), // x, positive forward
        Inches.of(12.4), // y, positive left
        Inches.of(21.75), // z, positive up
        new Rotation3d(
            Degrees.of(0), // roll, counterclockwise rotation angle around the X axis
            Degrees.of(0), // pitch, counterclockwise rotation angle around the y axis
            Degrees.of(0) // yaw, counterclockwise rotation angle around the z axis
        ));

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.01;
  }

  public static class ElevatorConstants {
    public static final int rightID = 13;
    public static final int leftID = 14;
    public static final LinearVelocity MAX_VELOCITY = FeetPerSecond.of(14);
    public static final LinearAcceleration MAX_ACCELERATION = FeetPerSecondPerSecond.of(6);
    public static final Distance L1 = Inches.of(18.6);
    public static final Distance L2 = Inches.of(23.6);
    public static final Distance L3 = Inches.of(38.14);
    public static final Distance L4 = Inches.of(64.10);
    public static final Distance A1 = Inches.of(40.0);
    public static final Distance A2 = Inches.of(54.7);
    public static final Distance P = Inches.of(20);
    public static final Distance CS = Inches.of(18.6);

    public static final double GEAR_RATIO = 16.0;
    public static final int GEAR_TEETH_NUM = 22;
    public static final int NUM_STAGES = 3;
    public static final Distance GEAR_PITCH = Inches.of(0.25);
    public static final Distance FLOOR_OFFSET = Inches.of(18.6);
    public static final Distance SPROCKET_PITCH_DIAMETER = Inches.of(1.9);
    public static final Distance CONVERSION_FACTOR = SPROCKET_PITCH_DIAMETER.times(Math.PI * NUM_STAGES / GEAR_RATIO);

    // control loop parameters
    // feedback constants
    public static final double kP = 50.0;
    public static final double kI = 0;
    public static final double kD = 0;
    // feedforward constants
    public static final double Ks = 0.31869;
    public static final double Kv = 4.0172;
    public static final double Ka = 0.74516;
    public static final double Kg = 0.76811;
  }

  public static class CoralConstants {
    public static final int pivotID = 15;
    public static final int intakeID = 16;
    public static final int encoderID = 0;
    public static final Angle ENCODER_OFFSET = Degrees.of(80);
    public static final double intakeSpeed = 0.3;
    public static final double ejectSpeed = 0.3;
    // feedforward constants
    public static final double Ks = 0.0;
    public static final double Kv = 0.0;
    public static final double Ka = 0.0;
    public static final double Kg = 0.4;
    // feedback constants
    public static final double kP = 7.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(900);
    public static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(720);
    public static final Angle UPPER_LIMIT = Degrees.of(65);
    public static final Angle LOWER_LIMIT = Degrees.of(-60);
    public static final double GEAR_RATIO = 16.0;
    public static final Angle STOWED = Degrees.of(50);
    public static final Angle L1 = Degrees.of(-13.0);
    public static final Angle L2 = Degrees.of(-30.5);
    public static final Angle L3 = Degrees.of(-30.5);
    public static final Angle L4 = Degrees.of(-46.75);
    public static final Angle A1 = Degrees.of(50.0);
    public static final Angle A2 = Degrees.of(50.0);
    public static final Angle P = Degrees.of(50.0);
    public static final Angle CS = Degrees.of(30.5);
  }

  public static class AlgaeConstants {
    public static final int pivotID = 17;
    public static final int intakeID = 18;
    public static final int encoderID = 1;
    public static final double INTAKE_SPEED = 0.8;
    public static final double EJECT_SPEED = 0.8;
    public static final double HOLD_SPEED = 0.10;
    // feedforward constants
    public static final double Ks = 0;
    public static final double Kg = 1.0;
    public static final double Kv = 0.25;
    public static final double Ka = 0;
    // feedback constants
    public static final double kP = 8.0;
    public static final double kI = 0;
    public static final double kD = 0.4;
    public static final double GEAR_RATIO = 16.0;
    public static final Angle ENCODER_OFFSET = Degrees.of(181.8);
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(720);
    public static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(540);
    public static final Angle L1 = Degrees.of(85);
    public static final Angle L2 = Degrees.of(85);
    public static final Angle L3 = Degrees.of(85);
    public static final Angle L4 = Degrees.of(85);
    public static final Angle A1 = Degrees.of(0);
    public static final Angle A2 = Degrees.of(0);
    public static final Angle P = Degrees.of(0);
    public static final Angle CS = Degrees.of(85);
  }

  public static class ClimbConstants {
    public static final int motorID = 19;
    public static final double speed = 0.95;
  }

  public static final double maxSpeed = Units.feetToMeters(10);
}
