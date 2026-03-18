/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.data;

import java.util.Locale;
import org.mtr.core.data.Data;
import org.mtr.core.data.Depot;
import org.mtr.core.data.Platform;
import org.mtr.core.data.RoutePlatformData;
import org.mtr.core.data.RouteType;
import org.mtr.core.data.Station;
import org.mtr.core.data.TransportMode;
import org.mtr.core.generated.data.RouteSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Utilities;
import org.mtr.legacy.data.DataFixer;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class Route
extends RouteSchema {
    public final ObjectArrayList<Depot> depots = new ObjectArrayList();
    public final LongArrayList durations = new LongArrayList();

    public Route(TransportMode transportMode, Data data) {
        super(transportMode, data);
    }

    public Route(ReaderBase readerBase, Data data) {
        super(DataFixer.convertRoute(readerBase), data);
        this.updateData(readerBase);
    }

    @Override
    public boolean isValid() {
        return !this.name.isEmpty();
    }

    public ObjectArrayList<RoutePlatformData> getRoutePlatforms() {
        return this.routePlatformData;
    }

    public String getRouteNumber() {
        return this.routeNumber;
    }

    public boolean getHidden() {
        return this.hidden;
    }

    public RouteType getRouteType() {
        return this.routeType;
    }

    public String getRouteTypeKey() {
        return String.format("%s_%s", new Object[]{this.transportMode, this.routeType}).toLowerCase(Locale.ENGLISH);
    }

    public CircularState getCircularState() {
        return this.circularState;
    }

    public String getDestination(int index) {
        String customDestination;
        for (int i = Math.min(this.routePlatformData.size() - 1, index); i >= 0 && !Route.destinationIsReset(customDestination = ((RoutePlatformData)this.routePlatformData.get(i)).getCustomDestination()); --i) {
            if (customDestination.isEmpty()) continue;
            return customDestination;
        }
        if (this.routePlatformData.isEmpty()) {
            return "";
        }
        Platform platform = null;
        if (this.circularState != CircularState.NONE) {
            for (int i = index + 1; !(i >= this.routePlatformData.size() || (platform = ((RoutePlatformData)this.routePlatformData.get((int)i)).platform) != null && platform.area != null && ((Station)platform.area).savedRails.stream().anyMatch(checkPlatform -> checkPlatform.routeColors.size() > 1 || !checkPlatform.routeColors.isEmpty() && !checkPlatform.routeColors.contains(this.getColor()))); ++i) {
            }
        }
        if (platform == null) {
            platform = ((RoutePlatformData)Utilities.getElement(this.routePlatformData, (int)-1)).platform;
        }
        return platform != null && platform.area != null ? ((Station)platform.area).getName() : "";
    }

    public void setRouteNumber(String routeNumber) {
        this.routeNumber = routeNumber;
    }

    public void setHidden(boolean hidden) {
        this.hidden = hidden;
    }

    public void setCircularState(CircularState circularState) {
        this.circularState = circularState;
    }

    public void setRouteType(RouteType routeType) {
        this.routeType = routeType;
    }

    public org.mtr.core.oba.Route getOBARouteElement() {
        return new org.mtr.core.oba.Route(this.getColorHex(), Utilities.formatName(this.routeNumber), Utilities.formatName(this.name), Utilities.formatName(this.name), this.getGtfsType(), this.getColorHex());
    }

    public static boolean destinationIsReset(String destination) {
        return destination.equals("\\r") || destination.equals("\\reset");
    }

    private int getGtfsType() {
        switch (this.transportMode) {
            case TRAIN: {
                return this.routeType == RouteType.LIGHT_RAIL ? 0 : 2;
            }
            case BOAT: {
                return 4;
            }
            case CABLE_CAR: {
                return 6;
            }
        }
        return 3;
    }

    public static enum CircularState {
        NONE(""),
        CLOCKWISE("\u21a9"),
        ANTICLOCKWISE("\u21aa");

        public final String emoji;

        private CircularState(String emoji) {
            this.emoji = emoji;
        }
    }
}

