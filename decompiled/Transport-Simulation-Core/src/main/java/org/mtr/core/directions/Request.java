/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2LongOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap
 */
package org.mtr.core.directions;

import java.util.function.Consumer;
import org.mtr.core.data.Position;
import org.mtr.core.directions.Connection;
import org.mtr.core.map.DirectionsResponse;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2LongOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;

public class Request {
    private final Position startPosition;
    private final Position endPosition;
    private final long startTime;
    private final Long2ObjectOpenHashMap<Connection> earliestConnections;
    private final Long2LongOpenHashMap walkingDistancesToEnd;
    private final Consumer<DirectionsResponse> callback;

    public Request(Position startPosition, Position endPosition, long startTime, Long2ObjectOpenHashMap<Connection> earliestConnections, Long2LongOpenHashMap walkingDistancesToEnd, Consumer<DirectionsResponse> callback) {
        this.startPosition = startPosition;
        this.endPosition = endPosition;
        this.startTime = startTime;
        this.earliestConnections = earliestConnections;
        this.walkingDistancesToEnd = walkingDistancesToEnd;
        this.callback = callback;
    }

    public Position startPosition() {
        return this.startPosition;
    }

    public Position endPosition() {
        return this.endPosition;
    }

    public long startTime() {
        return this.startTime;
    }

    public Long2ObjectOpenHashMap<Connection> earliestConnections() {
        return this.earliestConnections;
    }

    public Long2LongOpenHashMap walkingDistancesToEnd() {
        return this.walkingDistancesToEnd;
    }

    public Consumer<DirectionsResponse> callback() {
        return this.callback;
    }
}

