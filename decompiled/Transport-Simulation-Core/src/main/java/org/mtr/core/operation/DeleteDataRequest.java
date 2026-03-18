/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet
 */
package org.mtr.core.operation;

import javax.annotation.Nullable;
import org.mtr.core.data.NameColorDataBase;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.generated.operation.DeleteDataRequestSchema;
import org.mtr.core.operation.DeleteDataResponse;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet;

public final class DeleteDataRequest
extends DeleteDataRequestSchema {
    public DeleteDataRequest() {
    }

    public DeleteDataRequest(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public DeleteDataRequest addStationId(long stationId) {
        this.stationIds.add(stationId);
        return this;
    }

    public DeleteDataRequest addPlatformId(long platformId) {
        this.platformIds.add(platformId);
        return this;
    }

    public DeleteDataRequest addSidingId(long sidingId) {
        this.sidingIds.add(sidingId);
        return this;
    }

    public DeleteDataRequest addRouteId(long routeId) {
        this.routeIds.add(routeId);
        return this;
    }

    public DeleteDataRequest addDepotId(long depotId) {
        this.depotIds.add(depotId);
        return this;
    }

    public DeleteDataRequest addLiftFloorPosition(Position liftFloorPosition) {
        this.liftFloorPositions.add(liftFloorPosition);
        return this;
    }

    public DeleteDataRequest addRailId(String railId) {
        this.railIds.add(railId);
        return this;
    }

    public DeleteDataRequest addRailNodePosition(Position position) {
        this.railNodePositions.add(position);
        return this;
    }

    public DeleteDataResponse delete(Simulator simulator) {
        DeleteDataResponse deleteDataResponse = new DeleteDataResponse();
        ObjectArraySet<Position> railNodePositionsToUpdate = new ObjectArraySet<>();
        this.stationIds.forEach(stationId -> DeleteDataRequest.delete(stationId, simulator.stations, deleteDataResponse.getStationIds()));
        this.platformIds.forEach(platformId -> DeleteDataRequest.delete(platformId, simulator.platforms, deleteDataResponse.getPlatformIds()));
        this.sidingIds.forEach(sidingId -> DeleteDataRequest.delete(sidingId, simulator.sidings, deleteDataResponse.getSidingIds()));
        this.routeIds.forEach(routeId -> DeleteDataRequest.delete(routeId, simulator.routes, deleteDataResponse.getRouteIds()));
        this.depotIds.forEach(depotId -> DeleteDataRequest.delete(depotId, simulator.depots, deleteDataResponse.getDepotIds()));
        this.liftFloorPositions.forEach(liftPosition -> simulator.lifts.removeIf(lift -> {
            if (lift.getFloorIndex((Position)liftPosition) >= 0) {
                deleteDataResponse.getLiftIds().add(lift.getId());
                return true;
            }
            return false;
        }));
        this.railIds.forEach(railId -> DeleteDataRequest.delete((Rail)simulator.railIdMap.get(railId), (ObjectArraySet<Rail>)simulator.rails, railId, deleteDataResponse.getRailIds(), (ObjectArraySet<Position>)railNodePositionsToUpdate));
        this.railNodePositions.forEach(railNodePosition -> simulator.positionsToRail.getOrDefault(railNodePosition, new Object2ObjectOpenHashMap<>()).values().forEach(rail -> DeleteDataRequest.delete(rail, (ObjectArraySet<Rail>)simulator.rails, rail.getHexId(), deleteDataResponse.getRailIds(), railNodePositionsToUpdate)));
        simulator.sync();
        railNodePositionsToUpdate.forEach(railNodePosition -> {
            if (simulator.positionsToRail.getOrDefault(railNodePosition, new Object2ObjectOpenHashMap<>()).isEmpty()) {
                deleteDataResponse.getRailNodePositions().add(railNodePosition);
            }
        });
        return deleteDataResponse;
    }

    private static <T extends NameColorDataBase> void delete(long id, ObjectArraySet<T> dataSet, LongArrayList dataToUpdate) {
        if (dataSet.removeIf(data -> data.getId() == id)) {
            dataToUpdate.add(id);
        }
    }

    private static void delete(@Nullable Rail rail, ObjectArraySet<Rail> rails, String railId, ObjectArrayList<String> railsIdsToUpdate, ObjectArraySet<Position> railNodePositionsToUpdate) {
        if (rail != null) {
            rails.remove(rail);
            railsIdsToUpdate.add(railId);
            rail.writePositions(railNodePositionsToUpdate);
        }
    }
}

