/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.data;

import java.util.function.Consumer;
import javax.annotation.Nullable;
import org.mtr.core.data.Route;
import org.mtr.core.data.Siding;
import org.mtr.core.oba.Schedule;
import org.mtr.core.oba.SingleElement;
import org.mtr.core.oba.TripDetails;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public class Trip
implements Utilities {
    public final Route route;
    public final int routeIndex;
    public final int tripIndexInBlock;
    private final Siding siding;
    private final String tripIdPrefix;
    private final ObjectArrayList<StopTime> stopTimes = new ObjectArrayList();

    public Trip(Route route, int routeIndex, int tripIndexInBlock, Siding siding) {
        this.route = route;
        this.routeIndex = routeIndex;
        this.tripIndexInBlock = tripIndexInBlock;
        this.siding = siding;
        this.tripIdPrefix = String.format("%s_%s", siding.getHexId(), tripIndexInBlock);
    }

    public StopTime addStopTime(long startTime, long endTime, long platformId, int tripStopIndex, String customDestination) {
        StopTime stopTime = new StopTime(this, startTime, endTime, platformId, tripStopIndex, customDestination);
        this.stopTimes.add(stopTime);
        return stopTime;
    }

    public String getTripId(int departureIndex, long departureOffset) {
        return Utilities.concat(this.tripIdPrefix, "_", departureIndex, "_", departureOffset);
    }

    public void getUpcomingStopTimes(int tripStopIndex, ObjectArrayList<Trip> trips, boolean repeatIndefinitely, Consumer<StopTime> consumer) {
        boolean shouldBreak2;
        boolean shouldBreak1;
        StopTime stopTime1 = (StopTime)Utilities.getElement(this.stopTimes, tripStopIndex);
        if (stopTime1 == null) {
            return;
        }
        int tripsCount = trips.size();
        int tempTripIndex = this.tripIndexInBlock;
        int tempTripStopIndex = tripStopIndex + 1;
        do {
            Trip trip = (Trip)trips.get(tempTripIndex % tripsCount);
            if (tempTripStopIndex < trip.stopTimes.size()) {
                StopTime stopTime2 = (StopTime)trip.stopTimes.get(tempTripStopIndex);
                if (stopTime1.platformId != stopTime2.platformId) {
                    consumer.accept(stopTime2);
                    break;
                }
                ++tempTripStopIndex;
            } else {
                ++tempTripIndex;
                tempTripStopIndex = 0;
            }
            shouldBreak1 = !repeatIndefinitely && tempTripIndex >= tripsCount;
            boolean bl = shouldBreak2 = repeatIndefinitely && tempTripIndex >= this.tripIndexInBlock + tripsCount && tempTripStopIndex >= tripStopIndex;
        } while (!shouldBreak1 && !shouldBreak2);
    }

    public void getOBATripDetailsWithDataUsed(SingleElement<TripDetails> singleElement, long currentMillis, long offsetMillis, int departureIndex, long departureOffset, @Nullable Trip nextTrip, @Nullable Trip previousTrip) {
        if (this.stopTimes.isEmpty()) {
            return;
        }
        singleElement.addTrip(this.getOBATripElement(departureIndex, departureOffset));
        if (nextTrip != null) {
            singleElement.addTrip(nextTrip.getOBATripElement(departureIndex, departureOffset));
        }
        if (previousTrip != null) {
            singleElement.addTrip(previousTrip.getOBATripElement(departureIndex, departureOffset));
        }
        Schedule schedule = new Schedule(previousTrip == null ? "" : previousTrip.getTripId(departureIndex, departureOffset), nextTrip == null ? "" : nextTrip.getTripId(departureIndex, departureOffset), this.siding.getOBAFrequencyElement(currentMillis));
        this.stopTimes.forEach(tripStopTime -> {
            schedule.addStopTime(new org.mtr.core.oba.StopTime((StopTime)tripStopTime, offsetMillis));
            singleElement.addStop(tripStopTime.platformId);
        });
        singleElement.set(new TripDetails(this.getTripId(departureIndex, departureOffset), this.siding.getOBATripStatus(currentMillis, (StopTime)this.stopTimes.get(0), departureIndex, departureOffset, "", ""), schedule, this.siding.getOBAFrequencyElement(currentMillis)));
    }

    public org.mtr.core.oba.Trip getOBATripElement(String tripId, int departureIndex) {
        return new org.mtr.core.oba.Trip(this.route, tripId, departureIndex);
    }

    private org.mtr.core.oba.Trip getOBATripElement(int departureIndex, long departureOffset) {
        return this.getOBATripElement(this.getTripId(departureIndex, departureOffset), departureIndex);
    }

    public static class StopTime {
        public final Trip trip;
        public final long startTime;
        public final long endTime;
        public final long platformId;
        public final int tripStopIndex;
        public final String customDestination;

        private StopTime(Trip trip, long startTime, long endTime, long platformId, int tripStopIndex, String customDestination) {
            this.trip = trip;
            this.startTime = startTime;
            this.endTime = endTime;
            this.platformId = platformId;
            this.tripStopIndex = tripStopIndex;
            this.customDestination = customDestination;
        }
    }
}

