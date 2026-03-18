/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nonnull
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectSet
 */
package org.mtr.core.operation;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import org.mtr.core.data.ClientData;
import org.mtr.core.data.Data;
import org.mtr.core.data.Depot;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Rail;
import org.mtr.core.data.Route;
import org.mtr.core.data.Siding;
import org.mtr.core.data.SimplifiedRoute;
import org.mtr.core.data.Station;
import org.mtr.core.generated.operation.UpdateDataResponseSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.serializer.SerializedDataBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectSet;

public final class UpdateDataResponse
extends UpdateDataResponseSchema {
    private final Data data;

    public UpdateDataResponse(Data data) {
        this.data = data;
    }

    public UpdateDataResponse(ReaderBase readerBase, Data data) {
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
    protected Data routesDataParameter() {
        return this.data;
    }

    @Override
    @Nonnull
    protected Data depotsDataParameter() {
        return this.data;
    }

    public void write() {
        this.stations.forEach(station -> UpdateDataResponse.update(station, this.data.stations, this.data.stationIdMap.get(station.getId())));
        this.platforms.forEach(platform -> UpdateDataResponse.update(platform, this.data.platforms, this.data.platformIdMap.get(platform.getId())));
        this.sidings.forEach(siding -> UpdateDataResponse.update(siding, this.data.sidings, this.data.sidingIdMap.get(siding.getId())));
        LongArrayList hiddenRouteIds = new LongArrayList();
        this.routes.forEach(route -> {
            UpdateDataResponse.update(route, this.data.routes, this.data.routeIdMap.get(route.getId()));
            if (route.getHidden()) {
                hiddenRouteIds.add(route.getId());
            }
        });
        this.depots.forEach(depot -> UpdateDataResponse.update(depot, this.data.depots, this.data.depotIdMap.get(depot.getId())));
        this.rails.forEach(rail -> UpdateDataResponse.update(rail, this.data.rails, this.data.railIdMap.get(rail.getHexId())));
        if (this.data instanceof ClientData) {
            this.simplifiedRoutes.forEach(simplifiedRoute -> UpdateDataResponse.update(simplifiedRoute, ((ClientData)this.data).simplifiedRoutes, ((ClientData)this.data).simplifiedRoutes.stream().filter(existingSimplifiedRoute -> existingSimplifiedRoute.getId() == simplifiedRoute.getId()).findFirst().orElse(null)));
            ((ClientData)this.data).simplifiedRoutes.removeIf(simplifiedRoute -> hiddenRouteIds.contains(simplifiedRoute.getId()));
        }
        this.data.sync();
    }

    public void addDepot(Depot depot) {
        this.depots.add(depot);
    }

    ObjectArrayList<Station> getStations() {
        return this.stations;
    }

    ObjectArrayList<Platform> getPlatforms() {
        return this.platforms;
    }

    ObjectArrayList<Siding> getSidings() {
        return this.sidings;
    }

    ObjectArrayList<Route> getRoutes() {
        return this.routes;
    }

    ObjectArrayList<SimplifiedRoute> getSimplifiedRoutes() {
        return this.simplifiedRoutes;
    }

    ObjectArrayList<Depot> getDepots() {
        return this.depots;
    }

    ObjectArrayList<Rail> getRails() {
        return this.rails;
    }

    private static <T extends SerializedDataBase> void update(T newData, ObjectSet<T> dataSet, @Nullable T existingData) {
        if (existingData != null) {
            dataSet.remove(existingData);
        }
        dataSet.add(newData);
    }
}

