// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class VisionData {
    private Pose2d pose;
    private double timestamp;
    private Matrix<N3, N1> stddev;

    public VisionData(Pose2d pose, double timestamp, Matrix<N3, N1> stddev) {
        this.pose = pose;
        this.timestamp = timestamp;
        this.stddev = stddev;
    }

    public void updateVisionData(Pose2d pose, double timestamp, Matrix<N3, N1> stddev) {
        this.pose = pose;
        this.timestamp = timestamp;
        this.stddev = stddev;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getDataTimestamp() {
        return timestamp;
    }

    public Matrix<N3, N1> getStdMatrix() {
        return stddev;
    }
}
