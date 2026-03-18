/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.oba;

import java.util.TimeZone;
import org.mtr.core.data.Route;
import org.mtr.core.generated.oba.TripSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Utilities;

public final class Trip
extends TripSchema {
    public Trip(Route route, String id, int departureIndex) {
        super(route.getColorHex(), "1", id, Utilities.formatName(route.getName()), "", 0L, String.valueOf(departureIndex), "", Utilities.formatName(route.getRouteNumber()), TimeZone.getDefault().getID());
    }

    public Trip(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }
}

