/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.directions;

import javax.annotation.Nullable;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Position;
import org.mtr.core.data.Route;
import org.mtr.core.directions.DirectionsFinder;
import org.mtr.core.directions.IndependentConnection;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.RefreshableObject;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class Graph
extends RefreshableObject<Long2ObjectOpenHashMap<Long2ObjectOpenHashMap<IndependentConnection>>> {
    private final Long2ObjectOpenHashMap<Long2ObjectOpenHashMap<IndependentConnection>> independentConnections = new Long2ObjectOpenHashMap();
    private final Simulator simulator;

    public Graph(Simulator simulator) {
        super(new Long2ObjectOpenHashMap(), 30000L);
        this.simulator = simulator;
    }

    @Override
    @Nullable
    public Long2ObjectOpenHashMap<Long2ObjectOpenHashMap<IndependentConnection>> refresh(int currentRefreshStep) {
        if (currentRefreshStep == 0) {
            double gridSize = 250.0;
            Long2ObjectOpenHashMap<ObjectArrayList<Platform>> grid = new Long2ObjectOpenHashMap<>();
            this.simulator.platforms.forEach(platform -> {
                Position platformMidPosition = platform.getMidPosition();
                long gridKey = Graph.createGridKey(platformMidPosition.getX(), platformMidPosition.getZ(), 250.0);
                grid.computeIfAbsent(gridKey, key -> new ObjectArrayList<>()).add(platform);
            });
            this.simulator.platforms.forEach(platform -> {
                Position platformMidPosition = platform.getMidPosition();
                long platformId = platform.getId();
                int gridX = (int)Math.floor((double)platformMidPosition.getX() / 250.0);
                int gridZ = (int)Math.floor((double)platformMidPosition.getZ() / 250.0);
                for (int x = -1; x <= 1; ++x) {
                    for (int z = -1; z <= 1; ++z) {
                        long gridKey = Graph.createGridKey((double)(gridX + x) * 250.0, (double)(gridZ + z) * 250.0, 250.0);
                        ObjectArrayList<Platform> cell = grid.get(gridKey);
                        if (cell == null) continue;
                        for (Platform walkingPlatform : cell) {
                            long distance;
                            if (walkingPlatform.getId() == platformId || (distance = platformMidPosition.manhattanDistance(walkingPlatform.getMidPosition())) > 500L) continue;
                            this.independentConnections.computeIfAbsent(platformId, key -> new Long2ObjectOpenHashMap<>()).put(walkingPlatform.getId(), new IndependentConnection(null, platformId, walkingPlatform.getId(), Math.round((float)distance / 0.004f), distance));
                        }
                    }
                }
            });
            return null;
        }
        this.simulator.routes.forEach(route -> {
            if (route.getTransportMode().continuousMovement && !route.getHidden()) {
                DirectionsFinder.processRoute(route, route.getRoutePlatforms().size() - 1, (offsetTimeFromLastDeparture, duration, platform1, platform2) -> {
                    this.independentConnections.computeIfAbsent(platform1.getId(), key -> new Long2ObjectOpenHashMap<>()).put(platform2.getId(), new IndependentConnection((Route)route, platform1.getId(), platform2.getId(), duration, 0L));
                });
            }
        });
        return this.independentConnections;
    }

    private static long createGridKey(double x, double z, double gridSize) {
        long gridX = (long)Math.floor(x / gridSize);
        long gridZ = (long)Math.floor(z / gridSize);
        return gridX << 32 | gridZ & 0xFFFFFFFFL;
    }
}

