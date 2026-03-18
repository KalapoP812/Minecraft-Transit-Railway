/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 */
package org.mtr.core.oba;

import java.util.TimeZone;
import javax.annotation.Nullable;
import org.mtr.core.generated.oba.ScheduleSchema;
import org.mtr.core.oba.Frequency;
import org.mtr.core.oba.StopTime;
import org.mtr.core.serializer.ReaderBase;

public final class Schedule
extends ScheduleSchema {
    public Schedule(String previousTripId, String nextTripId, @Nullable Frequency frequency) {
        super(TimeZone.getDefault().getID(), previousTripId, nextTripId);
        this.frequency = frequency;
    }

    public Schedule(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public void addStopTime(StopTime stopTime) {
        this.stopTimes.add(stopTime);
    }

    @Override
    protected Frequency getDefaultFrequency() {
        return null;
    }
}

