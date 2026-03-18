/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 */
package org.mtr.core.directions;

import javax.annotation.Nullable;
import org.mtr.core.data.Route;

public class IndependentConnection {
    @Nullable
    private final Route route;
    private final long startPlatformId;
    private final long endPlatformId;
    private final long duration;
    private final long walkingDistance;

    public IndependentConnection(@Nullable Route route, long startPlatformId, long endPlatformId, long duration, long walkingDistance) {
        this.route = route;
        this.startPlatformId = startPlatformId;
        this.endPlatformId = endPlatformId;
        this.duration = duration;
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

    public long duration() {
        return this.duration;
    }

    public long walkingDistance() {
        return this.walkingDistance;
    }
}

