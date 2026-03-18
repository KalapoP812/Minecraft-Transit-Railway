/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 */
package org.mtr.core.oba;

import javax.annotation.Nullable;
import org.mtr.core.generated.oba.TripDetailsSchema;
import org.mtr.core.oba.Frequency;
import org.mtr.core.oba.Schedule;
import org.mtr.core.oba.TripStatus;

public final class TripDetails
extends TripDetailsSchema {
    public TripDetails(String tripId, TripStatus status, Schedule schedule, @Nullable Frequency frequency) {
        super(tripId, 0L, status, schedule);
        this.frequency = frequency;
    }

    @Override
    protected Frequency getDefaultFrequency() {
        return null;
    }
}

