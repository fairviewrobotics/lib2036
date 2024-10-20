package org.blackknightsrobotics.utils;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.geometry.Transform3d;
import org.blackknightsrobotics.vision.cameras.Camera;
import org.blackknightsrobotics.vision.cameras.LimelightCamera;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * CAN bus related utilities
 */
public class CANUtils {
    private static final Logger LOGGER = LoggerFactory.getLogger(CANUtils.class);

    /**
     * Configure a CAN Spark (max/flex) to free up CAN bus
     * @param motor The motor to configure
     * @return The configured motor
     */
    public static <T extends CANSparkBase> T configure(T motor) {
        return configure(motor, new int[] {
                1000, 20, 20, 1000, 1000, 1000, 500
        });
    }


    /**
     * Configure a CAN Spark motor
     * @param motor The motor to configure
     * @param rates A <code>int[]</code> with a length of 7 containing to delay in ms
     *              for each periodic status (0-6). <a href="https://docs.revrobotics.com/brushless/spark-max/control-interfaces">List of status</a>
     * @return The configured motor
     */
    public static <T extends CANSparkBase> T configure(T motor, int[] rates) {
        if (rates.length != 7) {
            LOGGER.error("Wrong length to configure can motor, expected 8 got {}", rates.length);
        }

        int i = 0;
        for (CANSparkLowLevel.PeriodicFrame frame : CANSparkLowLevel.PeriodicFrame.values()) {
            motor.setPeriodicFramePeriod(frame, rates[i]);
            i++;
        }

        return motor;
    }
}
