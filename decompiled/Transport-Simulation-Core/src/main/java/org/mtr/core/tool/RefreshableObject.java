/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 */
package org.mtr.core.tool;

import javax.annotation.Nullable;

public abstract class RefreshableObject<T> {
    private T data;
    private long expiryTime = 0L;
    private int currentRefreshStep = -1;
    private long totalRefreshTime = 0L;
    private long longestRefreshTime = 0L;
    private final long timeout;

    protected RefreshableObject(T initialValue, long timeout) {
        this.data = initialValue;
        this.timeout = timeout;
    }

    public final boolean tick() {
        long millis = System.currentTimeMillis();
        if (millis > this.expiryTime) {
            this.currentRefreshStep = 0;
            this.totalRefreshTime = 0L;
            this.longestRefreshTime = 0L;
            this.expiryTime = Long.MAX_VALUE;
        }
        if (this.currentRefreshStep >= 0) {
            T newData = this.refresh(this.currentRefreshStep);
            long refreshTime = System.currentTimeMillis() - millis;
            this.totalRefreshTime += refreshTime;
            this.longestRefreshTime = Math.max(this.longestRefreshTime, refreshTime);
            ++this.currentRefreshStep;
            if (newData != null) {
                this.data = newData;
                this.currentRefreshStep = -1;
                this.expiryTime = millis + this.timeout;
            }
            return true;
        }
        return false;
    }

    @Nullable
    public abstract T refresh(int var1);

    public T getData() {
        return this.data;
    }

    public long getTotalRefreshTime() {
        return this.totalRefreshTime;
    }

    public long getLongestRefreshTime() {
        return this.longestRefreshTime;
    }
}

