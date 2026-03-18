/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet
 */
package org.mtr.core.map;

import java.awt.Color;
import java.io.IOException;
import java.io.InputStream;
import java.util.function.Consumer;
import org.mtr.core.Main;
import org.mtr.core.data.AreaBase;
import org.mtr.core.data.SavedRailBase;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet;

public interface UpdateWebMap {
    public static final String MARKER_SET_STATIONS_ID = "mtr_stations";
    public static final String MARKER_SET_STATION_AREAS_ID = "mtr_station_areas";
    public static final String MARKER_SET_STATIONS_TITLE = "Stations";
    public static final String MARKER_SET_STATION_AREAS_TITLE = "Station Areas";
    public static final String MARKER_SET_DEPOTS_ID = "mtr_depots";
    public static final String MARKER_SET_DEPOT_AREAS_ID = "mtr_depot_areas";
    public static final String MARKER_SET_DEPOTS_TITLE = "Depots";
    public static final String MARKER_SET_DEPOT_AREAS_TITLE = "Depot Areas";
    public static final String STATION_ICON_PATH = "/assets/mtr/textures/block/sign/logo.png";
    public static final String DEPOT_ICON_PATH = "/assets/mtr/textures/block/sign/logo_grayscale.png";
    public static final String STATION_ICON_KEY = "mtr_station";
    public static final String DEPOT_ICON_KEY = "mtr_depot";
    public static final int ICON_SIZE = 24;

    public static void readResource(String path, Consumer<InputStream> callback) {
        try (InputStream inputStream = Main.class.getResourceAsStream(path);){
            if (inputStream != null) {
                callback.accept(inputStream);
            }
        }
        catch (IOException e) {
            Main.LOGGER.error("", (Throwable)e);
        }
    }

    public static <T extends AreaBase<T, U>, U extends SavedRailBase<U, T>> void iterateAreas(ObjectArraySet<T> areas, AreaCallback areaCallback) {
        areas.forEach(area -> {
            double x1 = area.getMinX();
            double z1 = area.getMinZ();
            double x2 = area.getMaxX() + 1L;
            double z2 = area.getMaxZ() + 1L;
            areaCallback.areaCallback(area.getHexId() + "_" + System.currentTimeMillis(), Utilities.formatName(area.getName()), new Color(area.getColor()), x1, z1, x2, z2, (x1 + x2) / 2.0, (z1 + z2) / 2.0);
        });
    }

    @FunctionalInterface
    public static interface AreaCallback {
        public void areaCallback(String var1, String var2, Color var3, double var4, double var6, double var8, double var10, double var12, double var14);
    }
}

