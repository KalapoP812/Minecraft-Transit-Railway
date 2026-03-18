/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.com.google.gson.JsonObject
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList
 */
package org.mtr.core.servlet;

import java.util.function.Consumer;
import org.mtr.core.Main;
import org.mtr.core.data.NameColorDataBase;
import org.mtr.core.map.Client;
import org.mtr.core.map.Clients;
import org.mtr.core.map.Departures;
import org.mtr.core.map.DirectionsRequest;
import org.mtr.core.map.StationAndRoutes;
import org.mtr.core.operation.ArrivalsRequest;
import org.mtr.core.serializer.JsonReader;
import org.mtr.core.servlet.CachedResponse;
import org.mtr.core.servlet.ServletBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.com.google.gson.JsonObject;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.Long2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList;

public final class SystemMapServlet
extends ServletBase {
    private final Object2ObjectAVLTreeMap<String, CachedResponse> stationsAndRoutesResponses = new Object2ObjectAVLTreeMap();
    private final Object2ObjectAVLTreeMap<String, CachedResponse> departuresResponses = new Object2ObjectAVLTreeMap();
    private final Object2ObjectAVLTreeMap<String, CachedResponse> clientsResponses = new Object2ObjectAVLTreeMap();

    public SystemMapServlet(ObjectImmutableList<Simulator> simulators) {
        super(simulators);
    }

    @Override
    public void getContent(String endpoint, String data, Object2ObjectAVLTreeMap<String, String> parameters, JsonReader jsonReader, Simulator simulator, Consumer<JsonObject> sendResponse) {
        if (endpoint.equals("directions")) {
            simulator.directionsFinder.addRequest(new DirectionsRequest(jsonReader, directionsResponse -> sendResponse.accept(Utilities.getJsonObjectFromData(directionsResponse))));
        } else {
            JsonObject jsonObject;
            switch (endpoint) {
                case "stations-and-routes": {
                    jsonObject = ((CachedResponse)this.stationsAndRoutesResponses.computeIfAbsent(simulator.dimension, key -> new CachedResponse(SystemMapServlet::getStationsAndRoutes, 30000L))).get(simulator);
                    break;
                }
                case "departures": {
                    jsonObject = ((CachedResponse)this.departuresResponses.computeIfAbsent(simulator.dimension, key -> new CachedResponse(SystemMapServlet::getDepartures, 3000L))).get(simulator);
                    break;
                }
                case "arrivals": {
                    jsonObject = Utilities.getJsonObjectFromData(new ArrivalsRequest(jsonReader).getArrivals(simulator));
                    break;
                }
                case "clients": {
                    jsonObject = ((CachedResponse)this.clientsResponses.computeIfAbsent(simulator.dimension, key -> new CachedResponse(SystemMapServlet::getClients, 3000L))).get(simulator);
                    break;
                }
                default: {
                    jsonObject = new JsonObject();
                }
            }
            sendResponse.accept(jsonObject);
        }
    }

    private static JsonObject getStationsAndRoutes(Simulator simulator) {
        StationAndRoutes stationAndRoutes = new StationAndRoutes(simulator.dimensions);
        simulator.stations.forEach(stationAndRoutes::addStation);
        simulator.routes.forEach(stationAndRoutes::addRoute);
        return Utilities.getJsonObjectFromData(stationAndRoutes);
    }

    private static JsonObject getDepartures(Simulator simulator) {
        long currentMillis = System.currentTimeMillis();
        Object2ObjectAVLTreeMap departures = new Object2ObjectAVLTreeMap();
        simulator.sidings.forEach(siding -> siding.getDeparturesForMap(currentMillis, (Object2ObjectAVLTreeMap<String, Long2ObjectAVLTreeMap<LongArrayList>>)departures));
        return Utilities.getJsonObjectFromData(new Departures(currentMillis, (Object2ObjectAVLTreeMap<String, Long2ObjectAVLTreeMap<LongArrayList>>)departures));
    }

    private static JsonObject getClients(Simulator simulator) {
        long currentMillis = System.currentTimeMillis();
        Object2ObjectAVLTreeMap clients = new Object2ObjectAVLTreeMap();
        simulator.clients.forEach(client -> {
            String clientId = client.uuid.toString();
            clients.put(clientId, new Client(clientId, Main.CLIENT_NAME_RESOLVER == null ? "" : Main.CLIENT_NAME_RESOLVER.apply(client.uuid), client.getPosition().getX(), client.getPosition().getZ(), simulator.stations.stream().filter(station -> station.inArea(client.getPosition())).map(NameColorDataBase::getHexId).findFirst().orElse("")));
        });
        simulator.sidings.forEach(siding -> siding.iterateVehiclesAndRidingEntities((vehicleExtraData, vehicleRidingEntity) -> {
            String clientId = vehicleRidingEntity.uuid.toString();
            Client client = (Client)clients.get(clientId);
            if (client != null) {
                clients.put(clientId, new Client(client, Utilities.numberToPaddedHexString(vehicleExtraData.getThisRouteId()), Utilities.numberToPaddedHexString(vehicleExtraData.getThisStationId()), Utilities.numberToPaddedHexString(vehicleExtraData.getNextStationId())));
            }
        }));
        return Utilities.getJsonObjectFromData(new Clients(currentMillis, (ObjectArrayList<Client>)new ObjectArrayList(clients.values())));
    }
}

