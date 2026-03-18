/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.map;

import org.mtr.core.generated.map.StationSchema;
import org.mtr.core.serializer.ReaderBase;

public final class Station
extends StationSchema {
    public Station(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    Station(org.mtr.core.data.Station station) {
        super(station.getHexId(), station.getName(), station.getColor(), station.getZone1(), station.getZone2(), station.getZone3());
        station.connectedStations.forEach(connectedStation -> this.connections.add(connectedStation.getHexId()));
    }
}

