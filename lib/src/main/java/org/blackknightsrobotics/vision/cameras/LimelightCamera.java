package org.blackknightsrobotics.vision.cameras;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import org.blackknightsrobotics.utils.NetworkTableUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Optional;

/**
 * Limelight  camera config
 */
public class LimelightCamera extends Camera {

    /**
     * Create a limelight camera
     * @param config A {@link CameraConfig} for the limelight
     */
    public LimelightCamera(CameraConfig config) {
        super(config);
        this.cameraNetworkTable = this.getCameraNetworkTable();
    }

    @Override
    public Optional<Pose3d> getPose() {
        if (!this.hasTarget()) return Optional.empty();

        switch (this.config.mode) {
            case ODOMETRY -> {
                double[] returnedPose = this.cameraNetworkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
                return Optional.of(new Pose3d(
                        new Translation3d(returnedPose[0], returnedPose[1], returnedPose[2]),
                        new Rotation3d(returnedPose[3], returnedPose[4], returnedPose[5])
                ));
            }
            case OBJECT -> {
                String key = this.objectPoseSuppliers.entrySet().iterator().next().getKey();
                return getObjectPose(key);
            }
        }

        return Optional.empty();
    }



    @Override
    public boolean hasTarget() {
        return this.cameraNetworkTable.getEntry("tv").getInteger(0) == 1;
    }

    @Override
    public double getLatency() {
        return this.cameraNetworkTable.getEntry("tl").getDouble(0)/1000 + this.cameraNetworkTable.getEntry("cl").getDouble(0)/1000;
    }

    @Override
    public double getDistanceFromTarget() {
        if (!this.hasTarget()) return -1;
        return this.cameraNetworkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[]{})[2];
    }

    @Override
    public NetworkTable getCameraNetworkTable() {
        return NetworkTableUtils.getNetworkTable(config.name).getTable();
    }
}
