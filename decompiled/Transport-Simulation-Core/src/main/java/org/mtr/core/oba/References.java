/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongAVLTreeSet
 */
package org.mtr.core.oba;

import org.mtr.core.data.Platform;
import org.mtr.core.data.Route;
import org.mtr.core.generated.oba.ReferencesSchema;
import org.mtr.core.oba.Agency;
import org.mtr.core.oba.Trip;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongAVLTreeSet;

public final class References
extends ReferencesSchema {
    private final IntAVLTreeSet routeColorsUsed = new IntAVLTreeSet();
    private final LongAVLTreeSet platformIdsUsed = new LongAVLTreeSet();

    public References(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    References() {
    }

    void addAgency(Agency agency) {
        this.agencies.add(agency);
    }

    void addRoute(int routeColor) {
        this.routeColorsUsed.add(routeColor);
    }

    void addStop(long platformId) {
        this.platformIdsUsed.add(platformId);
    }

    void addTrip(Trip trip) {
        this.trips.add(trip);
    }

    void build(Simulator simulator) {
        this.platformIdsUsed.forEach(platformId -> {
            Platform platform = (Platform)simulator.platformIdMap.get(platformId);
            if (platform != null) {
                this.stops.add(platform.getOBAStopElement(this.routeColorsUsed));
            }
        });
        this.routeColorsUsed.forEach(routeColor -> {
            for (Route route : simulator.routes) {
                if (route.getColor() != routeColor) continue;
                this.routes.add(route.getOBARouteElement());
                break;
            }
        });
    }
}

