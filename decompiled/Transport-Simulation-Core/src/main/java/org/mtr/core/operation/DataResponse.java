/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nonnull
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.operation;

import java.util.Collection;
import javax.annotation.Nonnull;
import org.mtr.core.data.ClientData;
import org.mtr.core.data.Data;
import org.mtr.core.data.Depot;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Rail;
import org.mtr.core.data.Route;
import org.mtr.core.data.Siding;
import org.mtr.core.data.SimplifiedRoute;
import org.mtr.core.data.Station;
import org.mtr.core.generated.operation.DataResponseSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class DataResponse
extends DataResponseSchema {
    private final Data data;

    DataResponse(Data data) {
        this.data = data;
    }

    public DataResponse(ReaderBase readerBase, ClientData data) {
        super(readerBase);
        this.data = data;
        this.updateData(readerBase);
    }

    @Override
    @Nonnull
    protected Data stationsDataParameter() {
        return this.data;
    }

    @Override
    @Nonnull
    protected Data platformsDataParameter() {
        return this.data;
    }

    @Override
    @Nonnull
    protected Data sidingsDataParameter() {
        return this.data;
    }

    @Override
    @Nonnull
    protected Data depotsDataParameter() {
        return this.data;
    }

    public void write() {
        if (!(!(this.data instanceof ClientData) || this.stations.isEmpty() && this.platforms.isEmpty() && this.sidings.isEmpty() && this.simplifiedRoutes.isEmpty() && this.depots.isEmpty() && this.rails.isEmpty())) {
            this.data.stations.removeIf(station -> !this.stationsToKeep.contains(station.getId()));
            this.data.stations.addAll(this.stations);
            this.data.platforms.removeIf(platform -> !this.platformsToKeep.contains(platform.getId()));
            this.data.platforms.addAll(this.platforms);
            this.data.sidings.removeIf(siding -> !this.sidingsToKeep.contains(siding.getId()));
            this.data.sidings.addAll(this.sidings);
            ((ClientData)this.data).simplifiedRoutes.removeIf(simplifiedRoute -> !this.simplifiedRoutesToKeep.contains(simplifiedRoute.getId()));
            ((ClientData)this.data).simplifiedRoutes.addAll(this.simplifiedRoutes);
            this.data.depots.removeIf(depot -> !this.depotsToKeep.contains(depot.getId()));
            this.data.depots.addAll(this.depots);
            this.data.rails.removeIf(rail -> !this.railsToKeep.contains(rail.getHexId()));
            this.data.rails.addAll(this.rails);
            this.data.sync();
        }
    }

    void addStation(Station station) {
        this.stations.add(station);
    }

    void addStation(long stationId) {
        this.stationsToKeep.add(stationId);
    }

    void addPlatform(Platform platform) {
        this.platforms.add(platform);
    }

    void addPlatform(long platformId) {
        this.platformsToKeep.add(platformId);
    }

    void addSiding(Siding siding) {
        this.sidings.add(siding);
    }

    void addSiding(long sidingId) {
        this.sidingsToKeep.add(sidingId);
    }

    void addDepot(Depot depot) {
        this.depots.add(depot);
    }

    void addDepot(long depotId) {
        this.depotsToKeep.add(depotId);
    }

    void addRoute(Route route) {
        SimplifiedRoute.addToList((ObjectArrayList<SimplifiedRoute>)this.simplifiedRoutes, route);
    }

    void addRoute(long routeId) {
        this.simplifiedRoutesToKeep.add(routeId);
    }

    void addRail(Rail rail) {
        this.rails.add(rail);
    }

    void addRail(String railId) {
        this.railsToKeep.add(railId);
    }
}

