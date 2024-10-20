package org.blackknightsrobotics.vision.cameras;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.Function;

/**
 * Abstract camera class
 */
public abstract class Camera {
    public CameraConfig config;

    protected NetworkTable cameraNetworkTable;

    protected static final Logger LOGGER = LoggerFactory.getLogger(Camera.class);

    protected HashMap<String, Function<NetworkTable, Pose3d>> objectPoseSuppliers = new HashMap<>();

    public Camera(CameraConfig config) {
        this.config = config;
    }

    /**
     * Return the pose of the robot in fieldspace if the robot {@link CameraConfig.Mode} is <code>ODOMETRY</code>
     * or the pose of an object relative to the robot if the mode is <code>OBJECT</code>
     * @return A {@link Pose3d} with the pose
     */
    public abstract Optional<Pose3d> getPose();

    /**
     * Does the camera have a target
     * @return Returns <code>true</code> if the camera has a target or <code>false</code> if there is no target
     */
    public abstract boolean hasTarget();

    /**
     * Get the latency in seconds of the vision processing
     * @return The latency in seconds
     */
    public abstract double getLatency();

    /**
     * Get the distance the robot is from the target
     * @return The distance from the target
     */
    public abstract double getDistanceFromTarget();

    /**
     * Get the cameras network table
     * @return The cameras {@link NetworkTable}
     */
    public abstract NetworkTable getCameraNetworkTable();

    /**
     * Get the {@link CameraConfig.Mode} of the camera
     * @return The mode
     */
    public CameraConfig.Mode getMode() {
        return this.config.mode;
    }

    /**
     * Get the name of the camera
     * @return The name of the camera
     */
    public String getName() {
        return this.config.name;
    }

    /**
     * Set the mode of the camera
     * @param mode The mode to set the camera to
     */
    public void setMode(CameraConfig.Mode mode) {
        this.config.mode = mode;
    }

    /**
     * Add a {@link Function} to get the pose of an object
     * @param name The name of the object/function to get the object
     * @param function A {@link Function}, the function is provided the
     *                 {@link NetworkTable} of the camera and should return a
     *                 {@link Pose3d} with the objects position relative to the robot
     */
    public void addObjectPoseFunction(String name, Function<NetworkTable, Pose3d> function) {
        objectPoseSuppliers.put(name, function); // TODO: Abstract NetworkTable for cameras to give photon vision and limelight the same api
    }

    /**
     * Get the pose of an object
     * @param objectName The name of the function to get the object as defined in <code>addObjectPoseFunction()</code>
     * @return
     */
    public Optional<Pose3d> getObjectPose(String objectName) {
        if (this.config.mode != CameraConfig.Mode.OBJECT) {
            LOGGER.warn("Incorrect usage! Use getPose() for non object detection");
            return Optional.empty();
        }

        if (!this.objectPoseSuppliers.containsKey(objectName)) {
            LOGGER.warn("No function for object '{}'", objectName);
            return Optional.empty();
        }

        return Optional.of(
                this.objectPoseSuppliers.get(objectName).apply(this.cameraNetworkTable)
        );
    }

    /**
     * Configuration object for cameras
     */
    public static class CameraConfig {
        public final double cutoffDist;

        public final double fov;

        public final Transform3d robotTransform;

        public final String name;

        public Mode mode;

        /**
         * Camera config constructor
         * @param name The name of the camera
         * @param mode The {@link Mode} of the camera
         * @param cutoffDist Distance in meters to stop using camera data
         * @param fov The fov of the camera
         * @param robotTransform A {@link Transform3d} The transform of the camera
         */
        private CameraConfig(String name, Mode mode, double cutoffDist, double fov, Transform3d robotTransform) {
            this.cutoffDist = cutoffDist;
            this.fov = fov;
            this.robotTransform = robotTransform;
            this.mode = mode;
            this.name = name;
        }

        /**
         * Camera modes
         */
        public enum Mode {
            ODOMETRY,
            OBJECT;
        }


        /**
         * Builder class for a {@link CameraConfig}
         */
        public static class CameraConfigBuilder {
            private final String name;
            private final Mode mode;

            private Transform3d robotTransform = new Transform3d();
            private double cutoffDistance = 3;
            private double fov = -1;

            /**
             * The base constructor for camera config builder
             * @param name Name of the camera
             * @param mode The camera mode
             */
            public CameraConfigBuilder(String name, Mode mode) {
                this.name = name;
                this.mode = mode;
            }

            /**
             * Set the transform of the camera to robot
             * @param robotTransform A {@link Transform3d} of the cameras transform to the robot
             * @return The {@link CameraConfigBuilder}
             */
            public CameraConfigBuilder setRobotTransform(Transform3d robotTransform) {
                this.robotTransform = robotTransform;
                return this;
            }

            /**
             * Set the fov of the camera
             * @param fov The diagonal fov of the camera
             * @return The {@link CameraConfigBuilder}
             */
            public CameraConfigBuilder setFov(double fov) {
                this.fov = fov;
                return this;
            }

            /**
             * Set the cutoff of the for the camera
             * @param cutoffDistance The cutoff to stop taking info from the camera (default: 3)
             * @return The {@link CameraConfigBuilder}
             */
            public CameraConfigBuilder setCutoffDistance(double cutoffDistance) {
                this.cutoffDistance = cutoffDistance;
                return this;
            }

            /**
             * Build a camera config
             * @return The created {@link CameraConfig}
             */
            public CameraConfig build() {
                return new CameraConfig(this.name, this.mode, this.cutoffDistance, this.fov, this.robotTransform);
            }
        }
    }

}
