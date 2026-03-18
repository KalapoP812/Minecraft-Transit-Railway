/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.operation;

import java.util.function.Consumer;
import org.mtr.core.data.ClientData;
import org.mtr.core.data.Data;
import org.mtr.core.data.Position;
import org.mtr.core.generated.operation.DeleteDataResponseSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class DeleteDataResponse
extends DeleteDataResponseSchema {
    public DeleteDataResponse() {
    }

    public DeleteDataResponse(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public void write(Data data) {
        data.stations.removeIf(station -> this.stationIds.contains(station.getId()));
        data.platforms.removeIf(platform -> this.platformIds.contains(platform.getId()));
        data.sidings.removeIf(siding -> this.sidingIds.contains(siding.getId()));
        data.routes.removeIf(route -> this.routeIds.contains(route.getId()));
        data.depots.removeIf(depot -> this.depotIds.contains(depot.getId()));
        data.lifts.removeIf(lift -> this.liftIds.contains(lift.getId()));
        data.rails.removeIf(rail -> this.railIds.contains(rail.getHexId()));
        if (data instanceof ClientData) {
            ((ClientData)data).simplifiedRoutes.removeIf(simplifiedRoute -> this.routeIds.contains(simplifiedRoute.getId()));
        }
        data.sync();
    }

    public void iterateRailNodePosition(Consumer<Position> consumer) {
        this.railNodePositions.forEach(consumer);
    }

    LongArrayList getStationIds() {
        return this.stationIds;
    }

    LongArrayList getPlatformIds() {
        return this.platformIds;
    }

    LongArrayList getSidingIds() {
        return this.sidingIds;
    }

    LongArrayList getRouteIds() {
        return this.routeIds;
    }

    LongArrayList getDepotIds() {
        return this.depotIds;
    }

    LongArrayList getLiftIds() {
        return this.liftIds;
    }

    ObjectArrayList<String> getRailIds() {
        return this.railIds;
    }

    ObjectArrayList<Position> getRailNodePositions() {
        return this.railNodePositions;
    }
}

