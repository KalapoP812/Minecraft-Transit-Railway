/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet
 */
package org.mtr.core.data;

import org.mtr.core.data.Data;
import org.mtr.core.data.Position;
import org.mtr.core.data.Route;
import org.mtr.core.data.Station;
import org.mtr.core.data.TransportMode;
import org.mtr.core.generated.data.PlatformSchema;
import org.mtr.core.oba.Stop;
import org.mtr.core.oba.StopDirection;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Angle;
import org.mtr.core.tool.EnumHelper;
import org.mtr.core.tool.LatLon;
import org.mtr.core.tool.Utilities;
import org.mtr.legacy.data.DataFixer;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet;

public final class Platform
extends PlatformSchema {
    public final ObjectAVLTreeSet<Route> routes = new ObjectAVLTreeSet();
    public final IntAVLTreeSet routeColors = new IntAVLTreeSet();
    private final Long2ObjectOpenHashMap<Angle> anglesFromDepot = new Long2ObjectOpenHashMap();

    public Platform(Position position1, Position position2, TransportMode transportMode, Data data) {
        super(position1, position2, transportMode, data);
    }

    public Platform(ReaderBase readerBase, Data data) {
        super(readerBase, data);
        this.updateData(readerBase);
        DataFixer.unpackPlatformDwellTime(readerBase, value -> {
            this.dwellTime = value;
        });
    }

    public void setDwellTime(long dwellTime) {
        this.dwellTime = dwellTime;
    }

    public long getDwellTime() {
        return this.transportMode.continuousMovement ? 1L : Math.max(1L, this.dwellTime);
    }

    public void setAngles(long depotId, Angle angle) {
        this.anglesFromDepot.put(depotId, angle);
    }

    public String getStationName() {
        return this.area == null ? "" : ((Station)this.area).getName();
    }

    public Stop getOBAStopElement(IntAVLTreeSet routesUsed) {
        Angle angle = null;
        for (Angle checkAngle : this.anglesFromDepot.values()) {
            if (angle == null) {
                angle = checkAngle;
                continue;
            }
            if (angle == checkAngle) continue;
            angle = null;
            break;
        }
        LatLon latLon = new LatLon(this.getMidPosition());
        String stationName = this.area == null ? "" : Utilities.formatName(((Station)this.area).getName());
        Stop stop = new Stop(this.getHexId(), this.getHexId(), String.format("%s%s%s%s", stationName, !stationName.isEmpty() && !this.name.isEmpty() ? " - " : "", this.name.isEmpty() ? "" : "Platform ", this.name), latLon.lat(), latLon.lon(), EnumHelper.valueOf(StopDirection.NONE, angle == null ? "" : angle.getClosest45().toString()));
        this.routeColors.forEach(color -> {
            stop.addRouteId(Utilities.numberToPaddedHexString(color, 6));
            routesUsed.add(color);
        });
        return stop;
    }
}

