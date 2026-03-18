/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 */
package org.mtr.core.directions;

import javax.annotation.Nullable;
import org.mtr.core.data.Route;

public class Connection {
    @Nullable
    private final Route route;
    private final long startPlatformId;
    private final long endPlatformId;
    private final long startTime;
    private final long endTime;
    private final long walkingDistance;

    public Connection(@Nullable Route route, long startPlatformId, long endPlatformId, long startTime, long endTime, long walkingDistance) {
        this.route = route;
        this.startPlatformId = startPlatformId;
        this.endPlatformId = endPlatformId;
        this.startTime = startTime;
        this.endTime = endTime;
        this.walkingDistance = walkingDistance;
    }

    public Route route() {
        return this.route;
    }

    public long startPlatformId() {
        return this.startPlatformId;
    }

    public long endPlatformId() {
        return this.endPlatformId;
    }

    public long startTime() {
        return this.startTime;
    }

    public long endTime() {
        return this.endTime;
    }

    public long walkingDistance() {
        return this.walkingDistance;
    }
}

