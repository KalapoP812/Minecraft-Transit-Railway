/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet
 */
package org.mtr.core.data;

import org.mtr.core.data.Data;
import org.mtr.core.data.SimplifiedRoute;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectAVLTreeSet;

public class ClientData
extends Data {
    public final ObjectAVLTreeSet<SimplifiedRoute> simplifiedRoutes = new ObjectAVLTreeSet();
    public final LongArrayList simplifiedRouteIds = new LongArrayList();

    @Override
    public void sync() {
        super.sync();
        this.simplifiedRouteIds.clear();
        this.simplifiedRoutes.forEach(simplifiedRoute -> this.simplifiedRouteIds.add(simplifiedRoute.getId()));
    }
}

