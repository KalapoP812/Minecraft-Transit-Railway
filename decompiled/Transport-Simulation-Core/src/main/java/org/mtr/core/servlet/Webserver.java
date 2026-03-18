/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.org.eclipse.jetty.server.Connector
 *  org.mtr.libraries.org.eclipse.jetty.server.Handler
 *  org.mtr.libraries.org.eclipse.jetty.server.Server
 *  org.mtr.libraries.org.eclipse.jetty.server.ServerConnector
 *  org.mtr.libraries.org.eclipse.jetty.servlet.ServletContextHandler
 *  org.mtr.libraries.org.eclipse.jetty.servlet.ServletHolder
 *  org.mtr.libraries.org.eclipse.jetty.util.thread.QueuedThreadPool
 *  org.mtr.libraries.org.eclipse.jetty.util.thread.ThreadPool
 */
package org.mtr.core.servlet;

import org.mtr.core.Main;
import org.mtr.libraries.org.eclipse.jetty.server.Connector;
import org.mtr.libraries.org.eclipse.jetty.server.Handler;
import org.mtr.libraries.org.eclipse.jetty.server.Server;
import org.mtr.libraries.org.eclipse.jetty.server.ServerConnector;
import org.mtr.libraries.org.eclipse.jetty.servlet.ServletContextHandler;
import org.mtr.libraries.org.eclipse.jetty.servlet.ServletHolder;
import org.mtr.libraries.org.eclipse.jetty.util.thread.QueuedThreadPool;
import org.mtr.libraries.org.eclipse.jetty.util.thread.ThreadPool;

public final class Webserver {
    private final Server server = new Server((ThreadPool)new QueuedThreadPool(100, 10, 120));
    private final ServerConnector serverConnector = new ServerConnector(this.server);
    private final ServletContextHandler servletContextHandler;

    public Webserver(int port) {
        this.server.setConnectors(new Connector[]{this.serverConnector});
        this.servletContextHandler = new ServletContextHandler();
        this.server.setHandler((Handler)this.servletContextHandler);
        this.serverConnector.setPort(port);
    }

    public void addServlet(ServletHolder servletHolder, String path) {
        this.servletContextHandler.addServlet(servletHolder, path);
    }

    public void start() {
        try {
            this.server.start();
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
        }
    }

    public void stop() {
        try {
            this.server.stop();
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
        }
        try {
            this.serverConnector.stop();
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
        }
    }
}

