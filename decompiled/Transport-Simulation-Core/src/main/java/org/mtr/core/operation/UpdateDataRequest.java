/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nonnull
 *  javax.annotation.Nullable
 *  org.mtr.libraries.com.google.gson.JsonElement
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectSet
 */
package org.mtr.core.operation;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import org.mtr.core.data.Data;
import org.mtr.core.data.Depot;
import org.mtr.core.data.Lift;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Rail;
import org.mtr.core.data.Route;
import org.mtr.core.data.Siding;
import org.mtr.core.data.SignalModification;
import org.mtr.core.data.SimplifiedRoute;
import org.mtr.core.data.Station;
import org.mtr.core.generated.operation.UpdateDataRequestSchema;
import org.mtr.core.operation.UpdateDataResponse;
import org.mtr.core.serializer.JsonReader;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.serializer.SerializedDataBase;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.com.google.gson.JsonElement;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectSet;

public final class UpdateDataRequest
extends UpdateDataRequestSchema {
    private final Data data;

    public UpdateDataRequest(Data data) {
        this.data = data;
    }

    public UpdateDataRequest(ReaderBase readerBase, Data data) {
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

    @Override
    @Nonnull
    protected Data liftsDataParameter() {
        return this.data;
    }

    public UpdateDataRequest addStation(Station station) {
        this.stations.add(station);
        return this;
    }

    public UpdateDataRequest addPlatform(Platform platform) {
        this.platforms.add(platform);
        return this;
    }

    public UpdateDataRequest addSiding(Siding siding) {
        this.sidings.add(siding);
        return this;
    }

    public UpdateDataRequest addRoute(Route route) {
        this.routes.add(route);
        return this;
    }

    public UpdateDataRequest addDepot(Depot depot) {
        this.depots.add(depot);
        return this;
    }

    public UpdateDataRequest addLift(Lift lift) {
        this.lifts.add(lift);
        return this;
    }

    public UpdateDataRequest addRail(Rail rail) {
        this.rails.add(rail);
        return this;
    }

    public UpdateDataRequest addSignalModification(SignalModification signalModification) {
        this.signalModifications.add(signalModification);
        return this;
    }

    public UpdateDataResponse update() {
        UpdateDataResponse updateDataResponse = new UpdateDataResponse(this.data);
        this.stations.forEach(station -> UpdateDataRequest.update(station, true, this.data.stationIdMap.get(station.getId()), this.data.stations, updateDataResponse.getStations()));
        this.platforms.forEach(platform -> UpdateDataRequest.update(platform, false, this.data.platformIdMap.get(platform.getId()), this.data.platforms, updateDataResponse.getPlatforms()));
        this.sidings.forEach(siding -> UpdateDataRequest.update(siding, false, this.data.sidingIdMap.get(siding.getId()), this.data.sidings, updateDataResponse.getSidings()));
        this.routes.forEach(route -> UpdateDataRequest.update(route, true, this.data.routeIdMap.get(route.getId()), this.data.routes, updateDataResponse.getRoutes()));
        this.depots.forEach(depot -> UpdateDataRequest.update(depot, true, this.data.depotIdMap.get(depot.getId()), this.data.depots, updateDataResponse.getDepots()));
        this.lifts.forEach(lift -> {
            UpdateDataRequest.getAndRemoveMatchingLifts(this.data, lift);
            UpdateDataRequest.update(lift, true, null, this.data.lifts, ObjectArrayList.of());
        });
        this.rails.forEach(rail -> UpdateDataRequest.update(rail, true, this.data.railIdMap.get(rail.getHexId()), this.data.rails, updateDataResponse.getRails()));
        this.signalModifications.forEach(signalModification -> signalModification.applyModificationToRail(this.data, updateDataResponse.getRails()));
        ObjectArrayList<Siding> sidingsToInit = new ObjectArrayList<>();
        updateDataResponse.getRails().forEach(rail -> rail.checkOrCreateSavedRail(this.data, updateDataResponse.getPlatforms(), sidingsToInit));
        this.data.sync();
        sidingsToInit.forEach(Siding::init);
        updateDataResponse.getSidings().addAll((ObjectList)sidingsToInit);
        updateDataResponse.getStations().forEach(station -> station.savedRails.forEach(platform -> platform.routes.forEach(route -> SimplifiedRoute.addToList(updateDataResponse.getSimplifiedRoutes(), route))));
        updateDataResponse.getPlatforms().forEach(platform -> platform.routes.forEach(route -> SimplifiedRoute.addToList(updateDataResponse.getSimplifiedRoutes(), route)));
        updateDataResponse.getRoutes().forEach(route -> {
            SimplifiedRoute.addToList(updateDataResponse.getSimplifiedRoutes(), route);
            route.getRoutePlatforms().forEach(routePlatformData -> routePlatformData.platform.routes.forEach(platformRoute -> SimplifiedRoute.addToList(updateDataResponse.getSimplifiedRoutes(), platformRoute)));
        });
        return updateDataResponse;
    }

    public static ObjectArrayList<Lift> getAndRemoveMatchingLifts(Data data, Lift lift) {
        ObjectArrayList liftsToModify = new ObjectArrayList();
        data.lifts.removeIf(existingLift -> {
            if (lift.overlappingFloors((Lift)existingLift)) {
                liftsToModify.add(existingLift);
                return true;
            }
            return false;
        });
        return liftsToModify;
    }

    private static <T extends SerializedDataBase> void update(T newData, boolean addNewData, @Nullable T existingData, ObjectSet<T> dataSet, ObjectArrayList<T> dataToUpdate) {
        boolean isValid;
        boolean isRail = newData instanceof Rail;
        boolean bl = isValid = !isRail || ((Rail)newData).isValid();
        if (existingData == null) {
            if (addNewData && isValid) {
                dataSet.add(newData);
                dataToUpdate.add(newData);
            }
        } else if (isValid) {
            if (isRail) {
                dataSet.remove(existingData);
                if (existingData instanceof Rail) {
                    ((Rail)newData).copySignalColors((Rail)existingData);
                }
                dataSet.add(newData);
                dataToUpdate.add(newData);
            } else {
                existingData.updateData(new JsonReader((JsonElement)Utilities.getJsonObjectFromData(newData)));
                dataToUpdate.add(existingData);
            }
        }
    }
}

