/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.data;

import org.mtr.core.data.InterchangeRouteNamesForColor;
import org.mtr.core.generated.data.InterchangeColorsForStationNameSchema;
import org.mtr.core.serializer.ReaderBase;

public class InterchangeColorsForStationName
extends InterchangeColorsForStationNameSchema {
    public InterchangeColorsForStationName(String stationName) {
        super(stationName);
    }

    public InterchangeColorsForStationName(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public void forEach(ColorConsumer consumer) {
        this.interchangeRouteNamesForColorList.sort((interchangeRouteNamesForColor1, interchangeRouteNamesForColor2) -> interchangeRouteNamesForColor2.getColor() - interchangeRouteNamesForColor1.getColor());
        this.interchangeRouteNamesForColorList.forEach(interchangeRouteNamesForColor -> consumer.accept(interchangeRouteNamesForColor.getColor(), (InterchangeRouteNamesForColor)interchangeRouteNamesForColor));
    }

    public void addColor(InterchangeRouteNamesForColor interchangeRouteNamesForColor) {
        this.interchangeRouteNamesForColorList.add(interchangeRouteNamesForColor);
    }

    String getStationName() {
        return this.stationName;
    }

    @FunctionalInterface
    public static interface ColorConsumer {
        public void accept(int var1, InterchangeRouteNamesForColor var2);
    }
}

