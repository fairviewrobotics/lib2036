package org.blackknightsrobotics.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;

public class NetworkTableUtils {
    private final NetworkTable table;
    private static final HashMap<String ,NetworkTableUtils> networkTableUtilsInstances = new HashMap<>();
    private static final Logger LOGGER = LoggerFactory.getLogger(NetworkTableUtils.class);

    public static NetworkTableUtils getNetworkTable(String table) {
        if (!networkTableUtilsInstances.containsKey(table)) {
            networkTableUtilsInstances.put(table, new NetworkTableUtils(table));
        }

        return networkTableUtilsInstances.get(table);
    }

    /**
     * This class is a utility class for interfacing with network tables
     *
     * @param table The string ID for a network table
     */
    private NetworkTableUtils(String table) {
        this.table = NetworkTableInstance.getDefault().getTable(table);
    }

    /**
     * This function returns the table of this instance of NetworkTableUtils
     *
     * @return Returns the table as a NetworkTable
     */
    public NetworkTable getTable() {
        return this.table;
    }

    /**
     * This function gets a value from network tables as a double
     *
     * @param key          The key in Network Tables for the value
     * @param defaultValue If the entry is not found or null we will return this
     * @return Returns the Network Table entry as a double
     */
    public Object getDouble(String key, double defaultValue) {
        return this.table.getEntry(key).getDouble(defaultValue);
    }

    /**
     * This function gets a value from network tables as a boolean
     *
     * @param key          The key in Network Tables for the value
     * @param defaultValue If the entry is not found or null we will return this
     * @return Returns the Network Table entry as a boolean
     */
    public Object getBoolean(String key, boolean defaultValue) {
        return this.table.getEntry(key).getBoolean(defaultValue);
    }

    /**
     * This function gets a value from network tables as a String
     *
     * @param key          The key in Network Tables for the value
     * @param defaultValue If the entry is not found or null we will return this
     * @return Returns the Network Table entry as a String
     */
    public Object getString(String key, String defaultValue) {
        return this.table.getEntry(key).getString(defaultValue);
    }

    /**
     * This function gets a value from Network Tables as a double array
     *
     * @param key     The key in Network Tables for the value
     * @param doubles If the entry is not found or null we will return this
     * @return Returns the Network Table entry as a double array (double[])
     */
    public double[] getDoubleArray(String key, double[] doubles) {
        return this.table.getEntry(key).getDoubleArray(doubles);
    }

    public void setDoubleArray(String key, double[] value) {
        this.table.getEntry(key).setDoubleArray(value);
    }

    /**
     * This function sets a double in network tables
     *
     * @param key   The key in Network Tables for the value
     * @param value What we are setting the entry to
     */
    public void setDouble(String key, double value) {
        this.table.getEntry(key).setDouble(value);
    }

    /**
     * This function sets a long in network tables
     *
     * @param key   The key in Network Tables for the value
     * @param value What we are setting the entry to
     */
    public void setLong(String key, long value) {
        this.table.getEntry(key).setInteger(value);
    }

    /**
     * This function sets a double in network tables
     *
     * @param key   The key in Network Tables for the value
     * @param value What we are setting the entry to
     */
    public void setBoolean(String key, boolean value) {
        this.table.getEntry(key).setBoolean(value);
    }

    /**
     * This function sets a String in network tables
     *
     * @param key   The key in Network Tables for the value
     * @param value What we are setting the entry to
     */
    public void setString(String key, String value) {
        this.table.getEntry(key).setString(value);
    }

    /**
     * This function returns a entry as whatever it is
     *
     * @param key          The key in Network Tables for the value
     * @param defaultValue We will return this is the entry is invalid
     * @return Returns the entry as whatever it is
     */
    @SuppressWarnings("unchecked")
    public <T> T get(String key, Class<T> type, T defaultValue) {
        if (type.equals(Boolean.class)) {
            return (T) getBoolean(key, (boolean) defaultValue);
        } else if (defaultValue instanceof Double) {
            return (T) getDouble(key, (double) defaultValue);
        } else if (defaultValue instanceof String) {
            return (T) getString(key, (String) defaultValue);
        } else {
            LOGGER.warn("Unknown type: {}", type.getSimpleName());
            return defaultValue;
        }
    }

    /**
     * Sets a entry in Network Tables to a value
     * @param key   The key in Network Tables for the value
     * @param value The value we are setting the entry to.
     */
    public <T> void setEntry(String key, Class<T> type, T value) {
        if (type.equals(Double.class)) {
            setDouble(key, (double) value);
        } else if (type.equals(String.class)) {
            setString(key, (String) value);
        } else if (type.equals(Boolean.class)) {
            setBoolean(key, (boolean) value);
        }
    }
}