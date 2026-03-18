/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.tool;

import org.mtr.core.data.Position;
import org.mtr.core.tool.Utilities;

public class LatLon {
    public final double lat;
    public final double lon;
    public static final double MAX_LAT = 90.0;
    public static final double MAX_LON = 180.0;
    private static final int EARTH_CIRCUMFERENCE_METERS = 40075017;

    public LatLon(double lat, double lon) {
        this.lat = lat;
        this.lon = lon;
    }

    public LatLon(Position position) {
        this(LatLon.metersToLat(-position.getZ()), LatLon.metersToLon(position.getX()));
    }

    public LatLon offset(double latOffset, double lonOffset) {
        return new LatLon(this.lat + latOffset, this.lon + lonOffset);
    }

    public double lat() {
        return this.lat;
    }

    public double lon() {
        return this.lon;
    }

    public static double metersToLat(double meters) {
        return Utilities.clamp(180.0 * meters / 4.0075017E7, -90.0, 90.0);
    }

    public static double metersToLon(double meters) {
        return Utilities.clamp(360.0 * meters / 4.0075017E7, -180.0, 180.0);
    }
}

