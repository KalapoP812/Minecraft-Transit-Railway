/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongListIterator
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectObjectImmutablePair
 */
package org.mtr.core.directions;

import java.util.Comparator;
import javax.annotation.Nullable;
import org.mtr.core.data.Route;
import org.mtr.core.directions.Connection;
import org.mtr.core.directions.DirectionsFinder;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.RefreshableObject;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongListIterator;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectObjectImmutablePair;

public final class Arrivals
extends RefreshableObject<ObjectArrayList<ObjectArrayList<Connection>>> {
    private long millis;
    private final Long2ObjectOpenHashMap<ObjectObjectImmutablePair<Route, LongArrayList>> tempDepartures = new Long2ObjectOpenHashMap();
    private final ObjectArrayList<ObjectArrayList<Connection>> routeConnectionsLists = new ObjectArrayList();
    private final Simulator simulator;

    public Arrivals(Simulator simulator) {
        super(new ObjectArrayList(), 5000L);
        this.simulator = simulator;
    }

    @Override
    @Nullable
    public ObjectArrayList<ObjectArrayList<Connection>> refresh(int currentRefreshStep) {
        int index1 = currentRefreshStep - 2;
        if (currentRefreshStep == 0) {
            this.tempDepartures.clear();
            this.routeConnectionsLists.clear();
            this.millis = this.simulator.getCurrentMillis();
            this.simulator.sidings.forEach(siding -> siding.getDeparturesForDirections(this.millis, this.tempDepartures));
            return null;
        }
        if (currentRefreshStep == 1) {
            this.tempDepartures.values().forEach(departuresForRoute -> DirectionsFinder.processRoute((Route)departuresForRoute.left(), ((Route)departuresForRoute.left()).getRoutePlatforms().size() - 1, (offsetTimeFromLastDeparture, duration, platform1, platform2) -> {
                LongListIterator longListIterator = ((LongArrayList)departuresForRoute.right()).iterator();
                while (longListIterator.hasNext()) {
                    long departureForRoute = (Long)longListIterator.next();
                    long vehicleArrival1 = departureForRoute - offsetTimeFromLastDeparture;
                    long vehicleArrival2 = vehicleArrival1 + duration;
                    if (vehicleArrival1 < this.millis) continue;
                    int index = (int)((vehicleArrival1 - this.millis) / 3600000L);
                    while (this.routeConnectionsLists.size() <= index) {
                        this.routeConnectionsLists.add(new ObjectArrayList());
                    }
                    ((ObjectArrayList)this.routeConnectionsLists.get(index)).add(new Connection((Route)departuresForRoute.left(), platform1.getId(), platform2.getId(), vehicleArrival1, vehicleArrival2, 0L));
                }
            }));
            return null;
        }
        if (index1 < this.routeConnectionsLists.size()) {
            ((ObjectArrayList)this.routeConnectionsLists.get(index1)).sort(Comparator.comparingLong(Connection::startTime));
            if (index1 == this.routeConnectionsLists.size() - 1) {
                return this.routeConnectionsLists;
            }
            return null;
        }
        return this.routeConnectionsLists;
    }
}

