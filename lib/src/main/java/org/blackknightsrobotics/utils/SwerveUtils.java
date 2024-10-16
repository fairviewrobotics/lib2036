package org.blackknightsrobotics.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class SwerveUtils {
    /**
     * Steps a value towards a target with a specified step size.
     * @param current The current or starting value. Can be positive or negative.
     * @param target The target value the algorithm will step towards. Can be positive or negative.
     * @param stepsize The maximum step size that can be taken.
     * @return The new value for `current` after performing the specified step towards the specified target.
     */
    public static double stepTowards(double current, double target, double stepsize) {
        if (Math.abs(current - target) <= stepsize) {
            return target;
        } else if (target < current) {
            return current - stepsize;
        } else {
            return current + stepsize;
        }
    }

    /**
     * Calculate an angle to make a pose face another pose
     * @param pose1 The pose to be rotated
     * @param pose2 The pose to face
     * @return The needed angle
     */
    public static double calcRotateAngle(Pose2d pose1, Pose2d pose2) {
        double xDiff = pose2.getX() - pose1.getX();
        double yDiff = pose2.getY() - pose1.getY();

        double angle = Math.atan2(yDiff, xDiff);

        return angle;
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param current The current or starting angle (in radians). Can lie outside the 0 to 2*PI range.
     * @param target The target angle (in radians) the algorithm will step towards. Can lie outside the 0 to 2*PI range.
     * @param stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for `current` after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    public static double stepTowardsCircular(double current, double target, double stepsize) {
        current = wrapAngle(current);
        target = wrapAngle(target);
        double stepDirection = Math.signum(target - current);
        double difference = Math.abs(current - target);
        if (difference <= stepsize) {
            return target;
        } else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (current + 2 * Math.PI - target < stepsize || target + 2 * Math.PI - current < stepsize) {
                return target;
            } else {
                return wrapAngle(current - stepDirection * stepsize); //this will handle wrapping gracefully
            }
        } else {
            return current + stepDirection * stepsize;
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param angleA An angle (in radians).
     * @param angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double AngleDifference(double angleA, double angleB) {
        double difference = Math.abs(angleA - angleB);
        return (difference > Math.PI) ? 2 * Math.PI - difference : difference;
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     * @param angle The angle (in radians) to wrap. Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    public static double wrapAngle(double angle) {
        double twoPi = 2 * Math.PI;
        if (angle == twoPi) {
            return 0.0;
        } else if (angle > twoPi) {
            return angle - twoPi * Math.floor(angle / twoPi);
        } else if (angle < 0.0) {
            return angle + twoPi * (Math.floor(-angle / twoPi) + 1);
        } else {
            return angle;
        }
    }
}