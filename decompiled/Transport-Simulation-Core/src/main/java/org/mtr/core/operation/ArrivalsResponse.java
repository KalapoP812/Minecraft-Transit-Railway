/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList
 */
package org.mtr.core.operation;

import org.mtr.core.generated.operation.ArrivalsResponseSchema;
import org.mtr.core.operation.ArrivalResponse;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList;

public final class ArrivalsResponse
extends ArrivalsResponseSchema {
    public ArrivalsResponse(long currentTime) {
        super(currentTime);
    }

    public ArrivalsResponse(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public long getCurrentTime() {
        return this.currentTime;
    }

    public ObjectImmutableList<ArrivalResponse> getArrivals() {
        return new ObjectImmutableList((ObjectList)this.arrivals);
    }

    public void add(ArrivalResponse arrivalResponse) {
        this.arrivals.add(arrivalResponse);
    }

    @FunctionalInterface
    public static interface ArrivalConsumer {
        public void apply(int var1, ArrivalResponse var2);
    }
}

