/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongCollection
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongConsumer
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongImmutableList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.operation;

import java.util.Collections;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Station;
import org.mtr.core.generated.operation.ArrivalsRequestSchema;
import org.mtr.core.operation.ArrivalResponse;
import org.mtr.core.operation.ArrivalsResponse;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongCollection;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongConsumer;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongImmutableList;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class ArrivalsRequest
extends ArrivalsRequestSchema {
    public ArrivalsRequest(LongImmutableList platformIds, int maxCountPerPlatform, int maxCountTotal) {
        super(maxCountPerPlatform, maxCountTotal);
        this.platformIds.addAll((LongList)platformIds);
    }

    public ArrivalsRequest(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public ArrivalsResponse getArrivals(Simulator simulator) {
        ObjectArrayList arrivalResponseList = new ObjectArrayList();
        ObjectAVLTreeSet visitedKeys = new ObjectAVLTreeSet();
        LongAVLTreeSet allPlatformIds = new LongAVLTreeSet();
        allPlatformIds.addAll((LongCollection)this.platformIds);
        this.platformIdsHex.forEach(platformIdHex -> allPlatformIds.add(ArrivalsRequest.parseHexId(platformIdHex)));
        this.stationIds.forEach(stationId -> ArrivalsRequest.iteratePlatformIds(simulator, stationId, arg_0 -> ((LongAVLTreeSet)allPlatformIds).add(arg_0)));
        this.stationIdsHex.forEach(stationIdHex -> ArrivalsRequest.iteratePlatformIds(simulator, ArrivalsRequest.parseHexId(stationIdHex), arg_0 -> ((LongAVLTreeSet)allPlatformIds).add(arg_0)));
        allPlatformIds.forEach(platformId -> {
            Platform platform = (Platform)simulator.platformIdMap.get(platformId);
            if (platform != null) {
                platform.routes.forEach(route -> route.depots.forEach(depot -> depot.savedRails.forEach(siding -> {
                    String key = String.format("%s_%s", platformId, siding.getId());
                    if (!visitedKeys.contains(key)) {
                        visitedKeys.add(key);
                        siding.getArrivals(simulator.getCurrentMillis(), platform, this.maxCountPerPlatform, (ObjectArrayList<ArrivalResponse>)arrivalResponseList);
                    }
                })));
            }
        });
        Collections.sort(arrivalResponseList);
        ArrivalsResponse arrivalsResponse = new ArrivalsResponse(simulator.getCurrentMillis());
        int i = 0;
        while ((long)i < (this.maxCountTotal <= 0L ? (long)arrivalResponseList.size() : Math.min((long)arrivalResponseList.size(), this.maxCountTotal))) {
            arrivalsResponse.add((ArrivalResponse)arrivalResponseList.get(i));
            ++i;
        }
        return arrivalsResponse;
    }

    private static long parseHexId(String id) {
        try {
            return Long.parseUnsignedLong(id, 16);
        }
        catch (Exception ignored) {
            return 0L;
        }
    }

    private static void iteratePlatformIds(Simulator simulator, long stationId, LongConsumer consumer) {
        Station station = (Station)simulator.stationIdMap.get(stationId);
        if (station != null) {
            station.savedRails.forEach(platform -> consumer.accept(platform.getId()));
        }
    }
}

