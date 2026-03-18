/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.Int2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.data;

import org.mtr.core.data.InterchangeRouteNamesForColor;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Route;
import org.mtr.core.data.RoutePlatformData;
import org.mtr.core.data.SimplifiedRoutePlatform;
import org.mtr.core.data.Station;
import org.mtr.core.generated.data.SimplifiedRouteSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.Int2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class SimplifiedRoute
extends SimplifiedRouteSchema
implements Comparable<SimplifiedRoute> {
    private SimplifiedRoute(Route route) {
        super(route.getId(), route.getName(), route.getColor(), route.getCircularState());
        for (int i = 0; i < route.getRoutePlatforms().size(); ++i) {
            Platform platform = ((RoutePlatformData)route.getRoutePlatforms().get((int)i)).platform;
            Station station = platform == null ? null : (Station)platform.area;
            Int2ObjectAVLTreeMap interchangeRoutes = new Int2ObjectAVLTreeMap();
            if (station == null) {
                if (platform != null) {
                    SimplifiedRoute.addInterchangeRoutes(route.getColor(), (Int2ObjectAVLTreeMap<InterchangeRouteNamesForColor>)interchangeRoutes, platform.routes);
                }
            } else {
                station.savedRails.forEach(stationPlatform -> SimplifiedRoute.addInterchangeRoutes(route.getColor(), (Int2ObjectAVLTreeMap<InterchangeRouteNamesForColor>)interchangeRoutes, stationPlatform.routes));
            }
            SimplifiedRoutePlatform simplifiedRoutePlatform = new SimplifiedRoutePlatform(platform == null ? 0L : platform.getId(), station == null ? 0L : station.getId(), route.getDestination(i), station == null ? "" : station.getName());
            interchangeRoutes.forEach((color, interchangeRouteNamesForColor) -> simplifiedRoutePlatform.addColor((InterchangeRouteNamesForColor)interchangeRouteNamesForColor));
            this.platforms.add(simplifiedRoutePlatform);
        }
    }

    public SimplifiedRoute(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public long getId() {
        return this.id;
    }

    public String getName() {
        return this.name;
    }

    public int getColor() {
        return (int)(this.color & 0xFFFFFFL);
    }

    public Route.CircularState getCircularState() {
        return this.circularState;
    }

    public ObjectArrayList<SimplifiedRoutePlatform> getPlatforms() {
        return this.platforms;
    }

    public int getPlatformIndex(long platformId) {
        for (int i = 0; i < this.platforms.size(); ++i) {
            if (((SimplifiedRoutePlatform)this.platforms.get(i)).getPlatformId() != platformId) continue;
            return i;
        }
        return -1;
    }

    public static void addToList(ObjectArrayList<SimplifiedRoute> simplifiedRoutes, Route route) {
        if (!route.getHidden()) {
            simplifiedRoutes.add(new SimplifiedRoute(route));
        }
    }

    private static void addInterchangeRoutes(int thisColor, Int2ObjectAVLTreeMap<InterchangeRouteNamesForColor> interchangeRoutes, ObjectAVLTreeSet<Route> routes) {
        routes.forEach(interchangeRoute -> {
            if (!interchangeRoute.getHidden() && interchangeRoute.getColor() != thisColor) {
                ((InterchangeRouteNamesForColor)interchangeRoutes.computeIfAbsent(interchangeRoute.getColor(), key -> new InterchangeRouteNamesForColor(interchangeRoute.getColor()))).addRouteName(interchangeRoute.getName().split("\\|\\|")[0]);
            }
        });
    }

    @Override
    public int compareTo(SimplifiedRoute simplifiedRoute) {
        return this.color == simplifiedRoute.color ? Long.compare(this.id, simplifiedRoute.id) : Long.compare(this.color, simplifiedRoute.color);
    }
}

