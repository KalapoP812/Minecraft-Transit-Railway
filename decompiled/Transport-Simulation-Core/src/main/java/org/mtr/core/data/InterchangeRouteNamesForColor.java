/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.data;

import java.util.Collections;
import java.util.function.Consumer;
import org.mtr.core.generated.data.InterchangeRouteNamesForColorSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class InterchangeRouteNamesForColor
extends InterchangeRouteNamesForColorSchema
implements Comparable<InterchangeRouteNamesForColor> {
    public InterchangeRouteNamesForColor(long color) {
        super(color);
    }

    public InterchangeRouteNamesForColor(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public void forEach(Consumer<String> consumer) {
        Collections.sort(this.routeNames);
        this.routeNames.forEach(consumer);
    }

    int getColor() {
        return (int)(this.color & 0xFFFFFFL);
    }

    void addRouteName(String routeName) {
        if (!this.routeNames.contains(routeName)) {
            this.routeNames.add(routeName);
        }
    }

    void addRouteNames(ObjectArrayList<String> routeNames) {
        this.routeNames.addAll(routeNames);
    }

    @Override
    public int compareTo(InterchangeRouteNamesForColor interchangeRouteNamesForColor) {
        return Long.compare(this.color, interchangeRouteNamesForColor.color);
    }
}

