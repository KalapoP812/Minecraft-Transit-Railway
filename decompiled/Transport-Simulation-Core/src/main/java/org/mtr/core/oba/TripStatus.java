/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 */
package org.mtr.core.oba;

import javax.annotation.Nullable;
import org.mtr.core.data.Trip;
import org.mtr.core.generated.oba.TripStatusSchema;
import org.mtr.core.oba.Frequency;
import org.mtr.core.oba.OccupancyStatus;
import org.mtr.core.oba.Position;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Utilities;

public final class TripStatus
extends TripStatusSchema
implements Utilities {
    public TripStatus(String tripId, Trip.StopTime stopTime, String closestStop, String nextStop, OccupancyStatus occupancyStatus, boolean predicted, long currentMillis, long deviation, String vehicleId, @Nullable Frequency frequency) {
        super(tripId, stopTime.trip.tripIndexInBlock, 0L, 0.0, 0.0, new Position(0.0, 0.0), 0.0, closestStop, 1L, nextStop, 1L, occupancyStatus, "", "default", predicted, currentMillis, currentMillis, new Position(0.0, 0.0), 0.0, 0.0, 0.0, deviation / 1000L, vehicleId);
        this.frequency = frequency;
    }

    public TripStatus(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    @Override
    protected Frequency getDefaultFrequency() {
        return null;
    }
}

