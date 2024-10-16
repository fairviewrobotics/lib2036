package org.blackknightsrobotics.utils;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;

/**
 * CAN bus related utilities
 */
public class CANUtils {
    /**
     * Configure a CAN Spark (max/flex) to free up CAN bus
     * @param motor The motor to configure
     * @return The configured motor
     */
    public static <T extends CANSparkBase> T configure(T motor) {
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 1000);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 1000);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 1000);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 1000);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 1000);
        return motor;
    }
}
