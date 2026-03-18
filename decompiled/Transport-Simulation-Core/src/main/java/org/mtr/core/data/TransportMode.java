/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.data;

import org.mtr.core.tool.Utilities;

public enum TransportMode {
    TRAIN(Integer.MAX_VALUE, false, true, true, true, 8),
    BOAT(1, false, true, true, true, 8),
    CABLE_CAR(1, true, false, false, false, 8),
    AIRPLANE(1, false, true, false, false, 16);

    public final int maxLength;
    public final boolean continuousMovement;
    public final boolean hasPitchAscending;
    public final boolean hasPitchDescending;
    public final boolean hasRouteTypeVariation;
    public final int stoppingSpace;
    public final long defaultSpeedKilometersPerHour;
    public final double defaultSpeedMetersPerMillisecond;

    private TransportMode(int maxLength, boolean continuousMovement, boolean hasPitchAscending, boolean hasPitchDescending, boolean hasRouteTypeVariation, int stoppingSpace) {
        this.maxLength = maxLength;
        this.continuousMovement = continuousMovement;
        this.hasPitchAscending = hasPitchAscending;
        this.hasPitchDescending = hasPitchDescending;
        this.hasRouteTypeVariation = hasRouteTypeVariation;
        this.stoppingSpace = stoppingSpace;
        this.defaultSpeedKilometersPerHour = continuousMovement ? 2L : 20L;
        this.defaultSpeedMetersPerMillisecond = Utilities.kilometersPerHourToMetersPerMillisecond(this.defaultSpeedKilometersPerHour);
    }
}

