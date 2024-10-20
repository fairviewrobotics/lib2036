package org.blackknightsrobotics.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;
import java.util.Vector;

public class PoseEstimatorWrapper extends SwerveDrivePoseEstimator {
    /**
     * Constructs a PoseEstimatorWrapper with default standard deviations for the model and vision
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
    public PoseEstimatorWrapper(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters) {
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    }

    /**
     * Constructs a PoseEstimatorWrapper.
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
    public PoseEstimatorWrapper(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters, stateStdDevs, visionMeasurementStdDevs);
    }

    /**
     * Constructs a PoseEstimatorWrapper.
     *
     * @param kinematics               A correctly-configured kinematics object for your drivetrain.
     * @param gyroAngle                The current gyro angle.
     * @param modulePositions          The current distance and rotation measurements of the swerve modules.
     * @param initialPoseMeters        The starting pose estimate.
     * @param wheelTrust               The trust of the wheels, increase this to trust them less
     * @param visionTrustTranslation   Trust of the vision for translation (increase to trust less)
     * @param visionTrustRotation      Trust of the vision for rotation (increase to trust less)
     */
    public PoseEstimatorWrapper(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters, double wheelTrust, double visionTrustTranslation, double visionTrustRotation) {
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters, VecBuilder.fill(wheelTrust, wheelTrust, wheelTrust), VecBuilder.fill(visionTrustTranslation, visionTrustTranslation, visionTrustRotation));
    }


    /**
     * Add a vision measurement from photon vision
     * @param estimatedRobotPose A {@link EstimatedRobotPose}
     */
    public void addVisionMeasurement(EstimatedRobotPose estimatedRobotPose) {
        this.addVisionMeasurement(
                estimatedRobotPose.estimatedPose.toPose2d(),
                estimatedRobotPose.timestampSeconds
        );
    }

    /**
     * Set the trust of vision
     * @param visionTrustTranslation Vision translation trust (set higher to decrease trust)
     * @param visionTrustRotation Vision rotation trust (set higher to decrease trust)
     */
    public void setVisionTrust(double visionTrustTranslation, double visionTrustRotation) {
        this.setVisionMeasurementStdDevs(VecBuilder.fill(visionTrustTranslation, visionTrustTranslation, visionTrustRotation));
    }

    /**
     * Set the trust of the wheels/gyro
     * @apiNote Not implemented, not sure if it is possible to set the wheel trust after initial configuration
     * @param wheelTrust Wheel/gyro trust set higher to trust wheels/gyro less
     */
    public void setWheelTrust(double wheelTrust) {
        // TODO: Implement, there is no easy way to do this that I can see
    }
}
