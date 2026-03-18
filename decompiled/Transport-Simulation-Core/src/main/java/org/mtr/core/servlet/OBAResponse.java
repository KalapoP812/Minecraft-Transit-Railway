/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.com.google.gson.JsonObject
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap
 */
package org.mtr.core.servlet;

import org.mtr.core.data.Platform;
import org.mtr.core.data.Siding;
import org.mtr.core.data.Station;
import org.mtr.core.oba.Agency;
import org.mtr.core.oba.AgencyWithCoverage;
import org.mtr.core.oba.ListElement;
import org.mtr.core.oba.SingleElement;
import org.mtr.core.oba.Stop;
import org.mtr.core.oba.StopWithArrivalsAndDepartures;
import org.mtr.core.oba.TripDetails;
import org.mtr.core.servlet.ResponseBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.LatLon;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.com.google.gson.JsonObject;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap;

public final class OBAResponse
extends ResponseBase<Object> {
    private final boolean includeReferences;
    private static final Agency AGENCY = new Agency();

    public OBAResponse(String data, Object2ObjectAVLTreeMap<String, String> parameters, long currentMillis, Simulator simulator) {
        super(data, parameters, new Object(), currentMillis, simulator);
        this.includeReferences = !"false".equals(parameters.get("includeReferences"));
    }

    public JsonObject getAgenciesWithCoverage() {
        ListElement<AgencyWithCoverage> listElement = ListElement.create(this.includeReferences, AGENCY);
        listElement.add(new AgencyWithCoverage());
        return listElement.toJson(this.simulator);
    }

    public JsonObject getAgency() {
        if (this.data.equals("1")) {
            SingleElement<Agency> singleElement = SingleElement.create(this.includeReferences, AGENCY);
            singleElement.set(AGENCY);
            return singleElement.toJson(this.simulator);
        }
        return null;
    }

    public JsonObject getArrivalsAndDeparturesForStop() {
        try {
            long platformId = Long.parseUnsignedLong(this.data, 16);
            Platform platform = (Platform)this.simulator.platformIdMap.get(platformId);
            SingleElement<StopWithArrivalsAndDepartures> singleElement = SingleElement.create(this.includeReferences, AGENCY);
            StopWithArrivalsAndDepartures stopWithArrivalsAndDepartures = new StopWithArrivalsAndDepartures(platform.getHexId());
            singleElement.set(stopWithArrivalsAndDepartures);
            singleElement.addStop(platformId);
            if (platform.area != null) {
                ((Station)platform.area).savedRails.forEach(nearbyPlatform -> {
                    if (nearbyPlatform.getId() != platformId) {
                        singleElement.addStop(nearbyPlatform.getId());
                        stopWithArrivalsAndDepartures.add(nearbyPlatform.getHexId());
                    }
                });
            }
            LongAVLTreeSet visitedSidingIds = new LongAVLTreeSet();
            platform.routes.forEach(route -> route.depots.forEach(depot -> depot.savedRails.forEach(siding -> {
                if (!visitedSidingIds.contains(siding.getId())) {
                    visitedSidingIds.add(siding.getId());
                    siding.getOBAArrivalsAndDeparturesElementsWithTripsUsed(singleElement, stopWithArrivalsAndDepartures, this.currentMillis, platform, Math.max(0, (int)this.getParameter("minutesBefore", 5.0)) * 60000, Math.max(0, (int)this.getParameter("minutesAfter", 35.0)) * 60000);
                }
            })));
            return singleElement.toJson(this.simulator);
        }
        catch (Exception exception) {
            return null;
        }
    }

    public JsonObject getStopsForLocation() {
        double lonSpan;
        double latSpan;
        LatLon latLon = this.getLatLonParameter();
        if (latLon == null) {
            return ListElement.create(this.includeReferences, AGENCY).toJson(this.simulator);
        }
        if (this.containsParameter("latSpan") && this.containsParameter("lonSpan")) {
            latSpan = Math.abs(this.getParameter("latSpan", 0.0)) / 2.0;
            lonSpan = Math.abs(this.getParameter("lonSpan", 0.0)) / 2.0;
        } else {
            double radius = this.getParameter("radius", 100.0);
            latSpan = LatLon.metersToLat(radius) / 2.0;
            lonSpan = LatLon.metersToLon(radius) / 2.0;
        }
        ListElement<Stop> listElement = ListElement.create(this.includeReferences, AGENCY);
        for (Platform platform : this.simulator.platforms) {
            LatLon platformLatLon = new LatLon(platform.getMidPosition());
            if (!Utilities.isBetween(platformLatLon.lat() - latLon.lat(), -latSpan, latSpan) || !Utilities.isBetween(platformLatLon.lon() - latLon.lon(), -lonSpan, lonSpan) || platform.routeColors.isEmpty()) continue;
            IntAVLTreeSet colorsUsed = new IntAVLTreeSet();
            if (!listElement.add(platform.getOBAStopElement(colorsUsed))) break;
            colorsUsed.forEach(listElement::addRoute);
        }
        return listElement.toJson(this.simulator);
    }

    public JsonObject getTripDetails() {
        String[] tripIdSplit = this.data.split("_");
        if (tripIdSplit.length == 4) {
            try {
                Siding siding = (Siding)this.simulator.sidingIdMap.get(Long.parseUnsignedLong(tripIdSplit[0], 16));
                if (siding != null) {
                    SingleElement<TripDetails> singleElement = SingleElement.create(this.includeReferences, AGENCY);
                    siding.getOBATripDetailsWithDataUsed(singleElement, this.currentMillis, Integer.parseInt(tripIdSplit[1]), Integer.parseInt(tripIdSplit[2]), Long.parseLong(tripIdSplit[3]));
                    return singleElement.toJson(this.simulator);
                }
            }
            catch (Exception exception) {
                // empty catch block
            }
        }
        return null;
    }

    private LatLon getLatLonParameter() {
        try {
            return new LatLon(Double.parseDouble((String)this.parameters.get("lat")), Double.parseDouble((String)this.parameters.get("lon")));
        }
        catch (Exception exception) {
            return null;
        }
    }

    private double getParameter(String name, double defaultValue) {
        try {
            return Double.parseDouble((String)this.parameters.get(name));
        }
        catch (Exception exception) {
            return defaultValue;
        }
    }

    private boolean containsParameter(String name) {
        return this.parameters.get(name) != null;
    }
}

