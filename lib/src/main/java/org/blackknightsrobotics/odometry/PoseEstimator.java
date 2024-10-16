package org.blackknightsrobotics.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoseEstimator extends SwerveDrivePoseEstimator {
    /**
     * Constructs a PoseEstimator with default standard deviations for the model and vision
     * measurements.
     *
     * <p>The default standard deviations of the model states are 0.1 meters for x, 0.1 meters for y,
     * and 0.1 radians for heading. The default standard deviations of the vision measurements are 0.9
     * meters for x, 0.9 meters for y, and 0.9 radians for heading.
     *
     * @param kinematics        A correctly-configured kinematics object for your drivetrain.
     * @param gyroAngle         The current gyro angle.
     * @param modulePositions   The current distance measurements and rotations of the swerve modules.
     * @param initialPoseMeters The starting pose estimate.
     */
    public PoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters) {
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    }

    /**
     * Constructs a PoseEstimator.
     *
     * @param kinematics               A correctly-configured kinematics object for your drivetrain.
     * @param gyroAngle                The current gyro angle.
     * @param modulePositions          The current distance and rotation measurements of the swerve modules.
     * @param initialPoseMeters        The starting pose estimate.
     * @param stateStdDevs             Standard deviations of the pose estimate (x position in meters, y position
     *                                 in meters, and heading in radians). Increase these numbers to trust your state estimate
     *                                 less.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     *                                 in meters, y position in meters, and heading in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public PoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters, stateStdDevs, visionMeasurementStdDevs);
    }

//    public PoseEstimator(SwerveDrivePoseEstimator kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters, double wheelTrust, double visionTrust) {
//        super(kinematics, gyroAngle, modulePositions, initialPoseMeters, );
//    }
}
