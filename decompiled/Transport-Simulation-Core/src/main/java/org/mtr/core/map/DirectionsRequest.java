/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.map;

import java.util.Arrays;
import java.util.UUID;
import java.util.function.Consumer;
import org.mtr.core.data.Client;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Position;
import org.mtr.core.data.Station;
import org.mtr.core.generated.map.DirectionsRequestSchema;
import org.mtr.core.map.DirectionsResponse;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;

public final class DirectionsRequest
extends DirectionsRequestSchema {
    public Consumer<DirectionsResponse> callback;

    public DirectionsRequest(Position startPosition, Position endPosition, long startTime, Consumer<DirectionsResponse> callback) {
        super(startTime);
        this.startPositionX = startPosition.getX();
        this.startPositionY = startPosition.getY();
        this.startPositionZ = startPosition.getZ();
        this.endPositionX = endPosition.getX();
        this.endPositionY = endPosition.getY();
        this.endPositionZ = endPosition.getZ();
        this.callback = callback;
    }

    public DirectionsRequest(ReaderBase readerBase, Consumer<DirectionsResponse> callback) {
        super(readerBase);
        this.updateData(readerBase);
        this.callback = callback;
    }

    public Position getStartPosition(Simulator simulator) {
        return this.getPosition(simulator, this.startPositionX, this.startPositionY, this.startPositionZ, this.startStationName, this.startClientId);
    }

    public Position getEndPosition(Simulator simulator) {
        return this.getPosition(simulator, this.endPositionX, this.endPositionY, this.endPositionZ, this.endStationName, this.endClientId);
    }

    public long getStartTime() {
        return this.startTime;
    }

    private Position getPosition(Simulator simulator, long x, long y, long z, String stationName, String clientId) {
        block7: {
            Station station;
            if (!stationName.isEmpty() && (station = (Station)simulator.stations.stream().filter(checkStation -> checkStation.getName().equalsIgnoreCase(stationName) || Arrays.stream(checkStation.getName().split("\\|")).anyMatch(namePart -> namePart.equalsIgnoreCase(stationName))).findFirst().orElse(null)) != null) {
                long xTotal = 0L;
                long yTotal = 0L;
                long zTotal = 0L;
                for (Platform platform : station.savedRails) {
                    Position position = platform.getMidPosition();
                    xTotal += position.getX();
                    yTotal += position.getY();
                    zTotal += position.getZ();
                }
                int count = station.savedRails.size();
                return new Position(xTotal / (long)count, yTotal / (long)count, zTotal / (long)count);
            }
            if (!clientId.isEmpty()) {
                try {
                    UUID uuid = UUID.fromString(clientId);
                    Platform platform = simulator.getNextPlatformOfRidingVehicle(uuid);
                    if (platform == null) {
                        for (Client client : simulator.clients) {
                            if (!client.uuid.equals(uuid)) continue;
                            return client.getPosition();
                        }
                        break block7;
                    }
                    return platform.getMidPosition();
                }
                catch (Exception exception) {
                    // empty catch block
                }
            }
        }
        return new Position(x, y, z);
    }
}

