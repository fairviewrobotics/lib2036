package org.blackknightsrobotics.vision.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import org.blackknightsrobotics.odometry.RobotTracker;
import org.blackknightsrobotics.utils.NetworkTableUtils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Optional;

/**
 * Photon vision camera
 */
public class PhotonVisionCamera extends Camera {
    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;

    private EstimatedRobotPose estimatedRobotPose;

    /**
     * Constructor for a photon vision camera
     * @param config A {@link CameraConfig} for the camera
     * @param layout A {@link AprilTagFieldLayout} for photon pose estimation
     */
    public PhotonVisionCamera(CameraConfig config, AprilTagFieldLayout layout) {
        super(config);
        this.photonCamera = new PhotonCamera(NetworkTableInstance.getDefault(), config.name);
        this.photonPoseEstimator = new PhotonPoseEstimator(layout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.photonCamera, config.robotTransform);
        this.cameraNetworkTable = this.photonCamera.getCameraTable();
    }

    /**
     * If you never want to use this camera for odometry then you don't need a {@link AprilTagFieldLayout}
     * @param config A {@link CameraConfig} for the camera
     */
    public PhotonVisionCamera(CameraConfig config) {
        super(config);
        if (this.config.mode == CameraConfig.Mode.ODOMETRY) {
            LOGGER.error("A field layout is required for odometry mode");
            throw new IllegalArgumentException("No field layout specified");
        }
        this.photonPoseEstimator = null;
        this.photonCamera = new PhotonCamera(NetworkTableInstance.getDefault(), config.name);
        this.cameraNetworkTable = this.photonCamera.getCameraTable();
    }

    @Override
    public Optional<Pose3d> getPose() {
        if (!this.hasTarget()) return Optional.empty();

        switch (this.config.mode) {
            case ODOMETRY -> {
                if (this.photonPoseEstimator == null) {
                    LOGGER.error("This photon camera can not be used for odometry, missing a field layout");
                    return Optional.empty();
                }
                if (RobotTracker.getInstance().isEmpty()) {
                    LOGGER.error("There is no instance of robot tracker");
                    return Optional.empty();
                }

                this.photonPoseEstimator.setReferencePose(RobotTracker.getInstance().get().getPose());
                this.photonPoseEstimator.update().ifPresent((erp) -> {
                    this.estimatedRobotPose = erp;
                });

                return Optional.ofNullable(this.estimatedRobotPose.estimatedPose);
            }
            case OBJECT -> {
                String key = this.objectPoseSuppliers.entrySet().iterator().next().getKey();
                return Optional.of(
                        this.objectPoseSuppliers.get(key).apply(this.cameraNetworkTable)
                );
            }
        }

        return Optional.empty();
    }

    @Override
    public boolean hasTarget() {
        return photonCamera.getLatestResult().hasTargets();
    }

    @Override
    public double getLatency() {
        return Timer.getFPGATimestamp() - this.estimatedRobotPose.timestampSeconds;
    }

    @Override
    public double getDistanceFromTarget() {
        Transform3d transform = this.photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget();
        return Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2) + Math.pow(transform.getZ(), 2));
    }

    @Override
    public NetworkTable getCameraNetworkTable() {
        return this.photonCamera.getCameraTable();
    }
}
