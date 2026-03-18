/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.directions;

import org.mtr.core.data.Platform;
import org.mtr.core.data.Route;
import org.mtr.core.data.RoutePlatformData;
import org.mtr.core.directions.Arrivals;
import org.mtr.core.directions.ConnectionScanAlgorithmProcessor;
import org.mtr.core.directions.Graph;
import org.mtr.core.map.DirectionsRequest;
import org.mtr.core.simulation.Simulator;

public final class DirectionsFinder {
    private final Graph graph;
    private final Arrivals arrivals;
    private final ConnectionScanAlgorithmProcessor connectionScanAlgorithmProcessor;
    public static final long MAX_WALKING_DISTANCE = 500L;
    public static final float WALKING_SPEED = 0.004f;

    public DirectionsFinder(Simulator simulator) {
        this.graph = new Graph(simulator);
        this.arrivals = new Arrivals(simulator);
        this.connectionScanAlgorithmProcessor = new ConnectionScanAlgorithmProcessor(this.graph, this.arrivals, simulator);
    }

    public void tick() {
        if (!this.connectionScanAlgorithmProcessor.directionsRequests.isEmpty()) {
            if (this.graph.tick()) {
                return;
            }
            if (this.arrivals.tick()) {
                return;
            }
        }
        this.connectionScanAlgorithmProcessor.tick();
    }

    public void addRequest(DirectionsRequest directionsRequest) {
        this.connectionScanAlgorithmProcessor.directionsRequests.add(directionsRequest);
    }

    public static void processRoute(Route route, int startIndex, RouteOffsetAndPlatformsCallback callback) {
        if (!route.getHidden() && route.durations.size() == route.getRoutePlatforms().size() - 1) {
            long totalOffset = ((RoutePlatformData)route.getRoutePlatforms().get((int)startIndex)).platform.getDwellTime();
            for (int i = startIndex - 1; i >= 0; --i) {
                Platform platform1 = ((RoutePlatformData)route.getRoutePlatforms().get((int)i)).platform;
                Platform platform2 = ((RoutePlatformData)route.getRoutePlatforms().get((int)(i + 1))).platform;
                long duration = route.durations.getLong(i);
                callback.accept(totalOffset += duration, duration, platform1, platform2);
                totalOffset += platform1.getDwellTime();
            }
        }
    }

    @FunctionalInterface
    public static interface RouteOffsetAndPlatformsCallback {
        public void accept(long var1, long var3, Platform var5, Platform var6);
    }
}

