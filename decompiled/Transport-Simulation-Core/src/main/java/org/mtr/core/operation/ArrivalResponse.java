/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.operation;

import java.util.function.Consumer;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Route;
import org.mtr.core.data.VehicleCar;
import org.mtr.core.generated.operation.ArrivalResponseSchema;
import org.mtr.core.operation.CarDetails;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class ArrivalResponse
extends ArrivalResponseSchema
implements Comparable<ArrivalResponse> {
    public ArrivalResponse(String destination, long arrival, long departure, long deviation, boolean realtime, long departureIndex, int stopIndex, Route route, Platform platform) {
        super(destination, arrival, departure, deviation, realtime, departureIndex, stopIndex == route.getRoutePlatforms().size() - 1, route.getId(), route.getName(), route.getRouteNumber(), route.getColor(), route.getCircularState(), platform.getId(), platform.getName());
    }

    public ArrivalResponse(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public String getDestination() {
        return this.destination;
    }

    public long getArrival() {
        return this.arrival;
    }

    public long getDeparture() {
        return this.departure;
    }

    public long getDeviation() {
        return this.deviation;
    }

    public boolean getRealtime() {
        return this.realtime;
    }

    public long getDepartureIndex() {
        return this.departureIndex;
    }

    public boolean getIsTerminating() {
        return this.isTerminating;
    }

    public long getRouteId() {
        return this.routeId;
    }

    public String getRouteName() {
        return this.routeName;
    }

    public String getRouteNumber() {
        return this.routeNumber;
    }

    public int getRouteColor() {
        return (int)(this.routeColor & 0xFFFFFFL);
    }

    public Route.CircularState getCircularState() {
        return this.circularState;
    }

    public long getPlatformId() {
        return this.platformId;
    }

    public String getPlatformName() {
        return this.platformName;
    }

    public int getCarCount() {
        return this.cars.size();
    }

    public void iterateCarDetails(Consumer<CarDetails> consumer) {
        this.cars.forEach(consumer);
    }

    public void setCarDetails(ObjectArrayList<VehicleCar> vehicleCars) {
        vehicleCars.forEach(vehicleCar -> this.cars.add(new CarDetails(vehicleCar.getVehicleId(), 0.0)));
    }

    @Override
    public int compareTo(ArrivalResponse arrivalResponse) {
        return Utilities.compare(this.arrival, arrivalResponse.arrival, () -> Utilities.compare(this.departureIndex, arrivalResponse.departureIndex, () -> Utilities.compare(this.platformName, arrivalResponse.platformName, () -> Utilities.compare(this.routeNumber, arrivalResponse.routeNumber, () -> Utilities.compare(this.destination, arrivalResponse.destination, () -> 0)))));
    }
}

