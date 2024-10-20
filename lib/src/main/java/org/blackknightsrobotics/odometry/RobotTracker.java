package org.blackknightsrobotics.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.blackknightsrobotics.config.ConfigManager;
import org.blackknightsrobotics.utils.NetworkTableUtils;
import org.blackknightsrobotics.vision.cameras.Camera;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


/**
 * Tracks the robots position and provides info about it
 */
public class RobotTracker {
    private static final Logger LOGGER = LoggerFactory.getLogger(RobotTracker.class);

    private static RobotTracker INSTANCE = null;

    private final PoseEstimatorWrapper poseEstimator;

    private final NetworkTableUtils NTRobotTracker = NetworkTableUtils.getNetworkTable("RobotTracker");

    private volatile boolean running = false;

    private volatile Pose2d pose = new Pose2d();

    private ScheduledExecutorService trackerScheduler;

    private final List<Camera> cameras = new ArrayList<>();

    /**
     * Robot Tracker constructor
     * @param kinematics A {@link SwerveDriveKinematics} object
     * @param gyroAngle The current angle of the gyro
     * @param modulePositions A list of {@link SwerveModulePosition}
     * @param initialPoseMeters The initial pose of the robot in meters
     * @param wheelTrust The trust of the wheels (higher = less trust)
     * @param visionTrustTranslation The trust of translation information from vision (higher = less trust)
     * @param visionTrustRotation The trust of the rotation information form vision (higher = less trust)
     */
    private RobotTracker(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters, double wheelTrust, double visionTrustTranslation, double visionTrustRotation) {
        this.poseEstimator = new PoseEstimatorWrapper(kinematics, gyroAngle, modulePositions, initialPoseMeters, wheelTrust, visionTrustTranslation, visionTrustRotation);
    }

    /**
     * Gets the current instance of robot tracker
     * @return Either the instance of robot tracker or an empty {@link Optional} if there is no instance of robot tracker
     */
    public static Optional<RobotTracker> getInstance() {
        if (INSTANCE == null) {
            LOGGER.error("There is no instance of robot tracker, use createInstance() to create one");
            return Optional.empty();
        }

        return Optional.of(INSTANCE);
    }

    /**
     * Create an instance of robot tracker
     *@param kinematics A {@link SwerveDriveKinematics} object
     * @param gyroAngle The current angle of the gyro
     * @param modulePositions A list of {@link SwerveModulePosition}
     * @param initialPoseMeters The initial pose of the robot in meters
     * @param wheelTrust The trust of the wheels (higher = less trust)
     * @param visionTrustTranslation The trust of translation information from vision (higher = less trust)
     * @param visionTrustRotation The trust of the rotation information form vision (higher = less trust)
     * @return The created instance of Robot Tracker
     */
    public static synchronized RobotTracker createInstance(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters, double wheelTrust, double visionTrustTranslation, double visionTrustRotation) {
        if (INSTANCE != null) {
            LOGGER.warn("In instance of robot tracker already exists, use getInstance() instead");
            return getInstance().get();
        }

        INSTANCE = new RobotTracker(kinematics, gyroAngle, modulePositions, initialPoseMeters, wheelTrust, visionTrustTranslation, visionTrustRotation);
        return INSTANCE;
    }

    /**
     * Add a camera for robot tracker to use
     * @param camera A {@link Camera}
     */
    public void addCamera(Camera camera) {
        this.cameras.add(camera);
    }

    /**
     * Initialize robot tracker this be run after cameras have been added
     */
    public void init() {
        if (running) {
            LOGGER.warn("Robot tracker is already running");
            return;
        }
        running = true;

        long pollRateMs = ConfigManager.getInstance().get("tracker_poll_rate_ms", Long.class, 20L);

        trackerScheduler = Executors.newScheduledThreadPool(1);

        trackerScheduler.scheduleAtFixedRate(() -> {
            while (running) {
                this.pose = this.poseEstimator.getEstimatedPosition();

                NTRobotTracker.setDoubleArray("Estimated Pose", new double[] {
                        this.pose.getX(),
                        this.pose.getY(),
                });

                for (Camera camera : cameras) {
                    if (camera.getMode() != Camera.CameraConfig.Mode.ODOMETRY || camera.getDistanceFromTarget() >= ConfigManager.getInstance().get(String.format("%s_odometry_cutoff_distance", camera.getName()), Double.class, camera.config.cutoffDist)) continue;
                    camera.getPose().ifPresent(pose -> {
                        this.poseEstimator.addVisionMeasurement(pose.toPose2d(), camera.getLatency());
                        LOGGER.debug("Added vision pose from {}", camera.getName());
                    });
                }
            }
        }, 0, pollRateMs, TimeUnit.MILLISECONDS);
    }

    /**
     * Update wheel info
     * @param gyroAngle The angle of the gyro
     * @param swerveModulePositions The positions of the swerve modules
     */
    public void update(Rotation2d gyroAngle, SwerveModulePosition[] swerveModulePositions) {
        poseEstimator.update(gyroAngle, swerveModulePositions);
    }

    /**
     * Shutdown the robot tracker
     */
    public void stop() {
        this.running = false;
        if (trackerScheduler != null) {
            trackerScheduler.shutdownNow();
            try {
                if (!trackerScheduler.awaitTermination(1, TimeUnit.SECONDS)) {
                    LOGGER.warn("Tracker scheduler did not terminate in the allocated time.");
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                LOGGER.error("Thread interrupted during tracker scheduler shutdown.");
            }
        }
    }

    /**
     * Get the current estimated pose of the robot
     * @return The current estimated pose
     */
    public Pose2d getPose() {
        return this.pose;
    }
}
