/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2LongOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectMap$Entry
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.directions;

import javax.annotation.Nullable;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Position;
import org.mtr.core.data.Station;
import org.mtr.core.directions.Arrivals;
import org.mtr.core.directions.Connection;
import org.mtr.core.directions.Graph;
import org.mtr.core.directions.Request;
import org.mtr.core.map.DirectionsConnection;
import org.mtr.core.map.DirectionsRequest;
import org.mtr.core.map.DirectionsResponse;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.RefreshableObject;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2LongOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class ConnectionScanAlgorithmProcessor
extends RefreshableObject<Object> {
    private Request[] requests = new Request[0];
    public final ObjectArrayList<DirectionsRequest> directionsRequests = new ObjectArrayList();
    private final Graph graph;
    private final Arrivals arrivals;
    private final Simulator simulator;
    private static final long START_PLATFORM_ID = -1L;
    private static final long END_PLATFORM_ID = -2L;

    public ConnectionScanAlgorithmProcessor(Graph graph, Arrivals arrivals, Simulator simulator) {
        super(new ObjectArrayList(), 0L);
        this.graph = graph;
        this.arrivals = arrivals;
        this.simulator = simulator;
    }

    @Override
    @Nullable
    public Object refresh(int currentRefreshStep) {
        int index1 = currentRefreshStep - 1;
        int index2 = index1 - ((ObjectArrayList)this.arrivals.getData()).size();
        if (currentRefreshStep == 0) {
            int directionsRequestCount = this.directionsRequests.size();
            if (directionsRequestCount == 0) {
                return 0;
            }
            this.requests = new Request[directionsRequestCount];
            long millis = System.currentTimeMillis();
            for (int i = 0; i < directionsRequestCount; ++i) {
                DirectionsRequest directionsRequest = (DirectionsRequest)this.directionsRequests.get(i);
                this.requests[i] = new Request(directionsRequest.getStartPosition(this.simulator), directionsRequest.getEndPosition(this.simulator), Math.max(millis, directionsRequest.getStartTime()), (Long2ObjectOpenHashMap<Connection>)new Long2ObjectOpenHashMap(), new Long2LongOpenHashMap(), directionsRequest.callback);
            }
            this.directionsRequests.clear();
            this.simulator.platforms.forEach(endPlatform -> {
                for (Request request : this.requests) {
                    long distanceToEnd;
                    Position endPosition = endPlatform.getMidPosition();
                    long distanceToStart = request.startPosition().manhattanDistance(endPosition);
                    if (distanceToStart <= 500L) {
                        long endTime = request.startTime() + (long)Math.round((float)distanceToStart / 0.004f);
                        request.earliestConnections().put(endPlatform.getId(), new Connection(null, -1L, endPlatform.getId(), request.startTime(), endTime, distanceToStart));
                        this.addIndependentConnectionsBFS(endPlatform.getId(), request.earliestConnections());
                    }
                    if ((distanceToEnd = request.endPosition().manhattanDistance(endPosition)) > 500L) continue;
                    request.walkingDistancesToEnd().put(endPlatform.getId(), distanceToEnd);
                }
            });
            return null;
        }
        if (index1 < ((ObjectArrayList)this.arrivals.getData()).size()) {
            ((ObjectArrayList<Connection>)((ObjectArrayList)this.arrivals.getData()).get(index1)).forEach(connection -> {
                for (Request request : this.requests) {
                    if (!ConnectionScanAlgorithmProcessor.addConnection(connection, request.earliestConnections())) continue;
                    this.addIndependentConnectionsBFS(connection.endPlatformId(), request.earliestConnections());
                }
            });
            return null;
        }
        if (index2 < this.requests.length) {
            Request request = this.requests[index2];
            DirectionsResponse directionsResponse = new DirectionsResponse(this.graph.getTotalRefreshTime(), this.arrivals.getTotalRefreshTime(), this.getTotalRefreshTime(), this.graph.getLongestRefreshTime(), this.arrivals.getLongestRefreshTime(), this.getLongestRefreshTime());
            Connection current = this.getEndConnection(request.earliestConnections(), request.walkingDistancesToEnd());
            while (current != null) {
                Platform startPlatform = current.startPlatformId() == -1L ? null : (Platform)this.simulator.platformIdMap.get(current.startPlatformId());
                Station startStation = startPlatform == null ? null : (Station)startPlatform.area;
                Platform endPlatform2 = current.endPlatformId() == -2L ? null : (Platform)this.simulator.platformIdMap.get(current.endPlatformId());
                Station endStation = endPlatform2 == null ? null : (Station)endPlatform2.area;
                directionsResponse.getDirectionsConnections().add(0, new DirectionsConnection(current.route() == null ? "" : current.route().getHexId(), startStation == null ? "" : startStation.getHexId(), endStation == null ? "" : endStation.getHexId(), startPlatform == null ? "" : startPlatform.getName(), endPlatform2 == null ? "" : endPlatform2.getName(), current.startTime(), current.endTime(), current.walkingDistance()));
                current = (Connection)request.earliestConnections().get(current.startPlatformId());
            }
            request.callback().accept(directionsResponse);
            if (index2 == this.requests.length - 1) {
                return 0;
            }
            return null;
        }
        return 0;
    }

    private void addIndependentConnectionsBFS(long lastPlatformId, Long2ObjectOpenHashMap<Connection> earliestConnections) {
        LongArrayList queue = new LongArrayList();
        queue.add(lastPlatformId);
        int index = 0;
        while (index < queue.size()) {
            long startPlatformId = queue.getLong(index++);
            Long2ObjectOpenHashMap<IndependentConnection> independentConnectionsForPlatformId = ((Long2ObjectOpenHashMap<Long2ObjectOpenHashMap<IndependentConnection>>)this.graph.getData()).get(startPlatformId);
            if (independentConnectionsForPlatformId == null) continue;
            Connection startConnection = (Connection)earliestConnections.get(startPlatformId);
            independentConnectionsForPlatformId.forEach((endPlatformId, independentConnection) -> {
                if (ConnectionScanAlgorithmProcessor.addConnection(new Connection(independentConnection.route(), startPlatformId, endPlatformId, startConnection.endTime(), startConnection.endTime() + independentConnection.duration(), independentConnection.walkingDistance()), earliestConnections)) {
                    queue.add(endPlatformId);
                }
            });
        }
    }

    @Nullable
    private Connection getEndConnection(Long2ObjectOpenHashMap<Connection> earliestConnections, Long2LongOpenHashMap walkingDistancesToEnd) {
        long bestPlatformId = 0L;
        long bestStartTime = 0L;
        long bestEndTime = Long.MAX_VALUE;
        long bestWalkingDistance = 0L;
        for (Long2ObjectMap.Entry entry : earliestConnections.long2ObjectEntrySet()) {
            long endTime;
            long platformId = entry.getLongKey();
            long startTime = ((Connection)entry.getValue()).startTime();
            long distance = walkingDistancesToEnd.getOrDefault(platformId, 501L);
            if (distance > 500L || (endTime = startTime + (long)Math.round((float)distance / 0.004f)) >= bestEndTime) continue;
            bestPlatformId = platformId;
            bestStartTime = startTime;
            bestEndTime = endTime;
            bestWalkingDistance = distance;
        }
        if (bestEndTime == Long.MAX_VALUE) {
            return null;
        }
        return new Connection(null, bestPlatformId, -2L, bestStartTime, bestEndTime, bestWalkingDistance);
    }

    private static boolean addConnection(Connection connection, Long2ObjectOpenHashMap<Connection> earliestConnections) {
        Connection startConnection = (Connection)earliestConnections.get(connection.startPlatformId());
        Connection endConnection = (Connection)earliestConnections.get(connection.endPlatformId());
        if (!(startConnection == null || startConnection.endTime() > connection.startTime() || endConnection != null && connection.endTime() >= endConnection.endTime() || startConnection.route() == null && connection.route() == null)) {
            earliestConnections.put(connection.endPlatformId(), connection);
            return true;
        }
        return false;
    }
}

