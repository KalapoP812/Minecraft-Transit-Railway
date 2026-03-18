/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongCollection
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.operation;

import java.util.Collection;
import java.util.UUID;
import org.mtr.core.data.Client;
import org.mtr.core.data.ClientData;
import org.mtr.core.data.Depot;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.data.Route;
import org.mtr.core.data.Siding;
import org.mtr.core.data.Station;
import org.mtr.core.generated.operation.DataRequestSchema;
import org.mtr.core.operation.DataResponse;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongCollection;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class DataRequest
extends DataRequestSchema {
    private final UUID uuid;

    public DataRequest(UUID uuid, Position clientPosition, long requestRadius) {
        super(uuid.toString(), clientPosition, requestRadius);
        this.uuid = uuid;
    }

    public DataRequest(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
        this.uuid = UUID.fromString(this.clientId);
    }

    public DataResponse getData(Simulator simulator) {
        DataResponse dataResponse = new DataResponse(simulator);
        LongAVLTreeSet addedStationIds = new LongAVLTreeSet();
        LongAVLTreeSet addedPlatformIds = new LongAVLTreeSet();
        LongAVLTreeSet addedRouteIds = new LongAVLTreeSet();
        simulator.stations.forEach(station -> {
            if (station.inArea(this.clientPosition, this.requestRadius)) {
                ObjectArrayList<Station> stationsToAdd = new ObjectArrayList<>();
                stationsToAdd.add(station);
                stationsToAdd.addAll(station.connectedStations);
                stationsToAdd.forEach(addStation -> {
                    if (!addedStationIds.contains(addStation.getId())) {
                        if (this.existingStationIds.contains(addStation.getId())) {
                            dataResponse.addStation(addStation.getId());
                        } else {
                            dataResponse.addStation((Station)addStation);
                        }
                        addedStationIds.add(addStation.getId());
                        addStation.savedRails.forEach(platform -> this.addPlatform(platform, dataResponse, addedPlatformIds, addedRouteIds));
                    }
                });
            }
        });
        simulator.platforms.forEach(platform -> {
            if (platform.closeTo(this.clientPosition, this.requestRadius)) {
                this.addPlatform((Platform)platform, dataResponse, addedPlatformIds, addedRouteIds);
            }
        });
        simulator.sidings.forEach(siding -> {
            if (siding.closeTo(this.clientPosition, this.requestRadius)) {
                if (this.existingSidingIds.contains(siding.getId())) {
                    dataResponse.addSiding(siding.getId());
                } else {
                    dataResponse.addSiding((Siding)siding);
                }
            }
        });
        simulator.depots.forEach(depot -> {
            if (depot.inArea(this.clientPosition, this.requestRadius)) {
                if (this.existingDepotIds.contains(depot.getId())) {
                    dataResponse.addDepot(depot.getId());
                } else {
                    dataResponse.addDepot((Depot)depot);
                }
            }
        });
        simulator.railIdMap.forEach((railId, rail) -> {
            if (rail.closeTo(this.clientPosition, this.requestRadius)) {
                if (this.existingRailIds.contains(railId)) {
                    dataResponse.addRail(rail.getHexId());
                } else {
                    dataResponse.addRail((Rail)rail);
                }
            }
        });
        boolean createClient = true;
        for (Client client : simulator.clients) {
            if (!client.uuid.equals(this.uuid)) continue;
            client.setPositionAndUpdateRadius(this.clientPosition, this.requestRadius);
            createClient = false;
            break;
        }
        if (createClient) {
            Client client = new Client(this.uuid);
            client.setPositionAndUpdateRadius(this.clientPosition, this.requestRadius);
            simulator.clients.add(client);
        }
        return dataResponse;
    }

    public void writeExistingIds(ClientData clientData) {
        this.existingStationIds.addAll((LongCollection)clientData.stationIdMap.keySet());
        this.existingPlatformIds.addAll((LongCollection)clientData.platformIdMap.keySet());
        this.existingSidingIds.addAll((LongCollection)clientData.sidingIdMap.keySet());
        this.existingSimplifiedRouteIds.addAll((LongList)clientData.simplifiedRouteIds);
        this.existingDepotIds.addAll((LongCollection)clientData.depotIdMap.keySet());
        this.existingRailIds.addAll(clientData.railIdMap.keySet());
    }

    private void addPlatform(Platform platform, DataResponse dataResponse, LongAVLTreeSet addedPlatformIds, LongAVLTreeSet addedRouteIds) {
        if (!addedPlatformIds.contains(platform.getId())) {
            if (this.existingPlatformIds.contains(platform.getId())) {
                dataResponse.addPlatform(platform.getId());
            } else {
                dataResponse.addPlatform(platform);
            }
            addedPlatformIds.add(platform.getId());
            platform.routes.forEach(route -> {
                if (!route.getHidden() && !addedRouteIds.contains(route.getId())) {
                    if (this.existingSimplifiedRouteIds.contains(route.getId())) {
                        dataResponse.addRoute(route.getId());
                    } else {
                        dataResponse.addRoute((Route)route);
                    }
                    addedRouteIds.add(route.getId());
                }
            });
        }
    }
}

