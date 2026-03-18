/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  javax.annotation.ParametersAreNonnullByDefault
 *  org.apache.logging.log4j.LogManager
 *  org.apache.logging.log4j.Logger
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList
 *  org.mtr.libraries.javax.servlet.Servlet
 *  org.mtr.libraries.org.eclipse.jetty.servlet.ServletHolder
 */
package org.mtr.core;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Locale;
import java.util.UUID;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Function;
import javax.annotation.Nullable;
import javax.annotation.ParametersAreNonnullByDefault;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.mtr.core.data.Depot;
import org.mtr.core.generated.WebserverResources;
import org.mtr.core.servlet.OBAServlet;
import org.mtr.core.servlet.QueueObject;
import org.mtr.core.servlet.SystemMapServlet;
import org.mtr.core.servlet.WebServlet;
import org.mtr.core.servlet.Webserver;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList;
import org.mtr.libraries.javax.servlet.Servlet;
import org.mtr.libraries.org.eclipse.jetty.servlet.ServletHolder;

@ParametersAreNonnullByDefault
public class Main {
    private final ObjectImmutableList<Simulator> simulators;
    @Nullable
    private final Webserver webserver;
    @Nullable
    private final ScheduledExecutorService scheduledExecutorService;
    @Nullable
    public static Function<UUID, String> CLIENT_NAME_RESOLVER;
    public static final Logger LOGGER;
    public static final int MILLISECONDS_PER_TICK = 10;

    public static void main(String[] args) {
        try {
            int i = 0;
            Path rootPath = Paths.get(args[i++], new String[0]);
            int webserverPort = Integer.parseInt(args[i++]);
            boolean threadedSimulation = Boolean.parseBoolean(args[i++]);
            boolean threadedFileLoading = Boolean.parseBoolean(args[i++]);
            String[] dimensions = new String[args.length - i];
            System.arraycopy(args, i, dimensions, 0, dimensions.length);
            Main main = new Main(rootPath, webserverPort, threadedSimulation, threadedFileLoading, null, dimensions);
            main.readConsoleInput();
        }
        catch (Exception e) {
            Main.printHelp();
            LOGGER.error("", (Throwable)e);
        }
    }

    public Main(Path rootPath, int webserverPort, boolean threadedSimulation, boolean threadedFileLoading, @Nullable Consumer<Webserver> additionalWebserverSetup, String ... dimensions) {
        ObjectArrayList tempSimulators = new ObjectArrayList();
        LOGGER.info("Loading files...");
        for (String dimension : dimensions) {
            tempSimulators.add(new Simulator(dimension, dimensions, rootPath, threadedFileLoading));
        }
        this.simulators = new ObjectImmutableList((ObjectList)tempSimulators);
        if (webserverPort > 0) {
            this.webserver = new Webserver(webserverPort);
            this.webserver.addServlet(new ServletHolder((Servlet)new MainWebServlet(WebserverResources::get, "/")), "/");
            this.webserver.addServlet(new ServletHolder((Servlet)new SystemMapServlet(this.simulators)), "/mtr/api/map/*");
            this.webserver.addServlet(new ServletHolder((Servlet)new OBAServlet(this.simulators)), "/oba/api/where/*");
            if (additionalWebserverSetup != null) {
                additionalWebserverSetup.accept(this.webserver);
            }
            this.webserver.start();
        } else {
            this.webserver = null;
        }
        if (threadedSimulation) {
            this.scheduledExecutorService = Executors.newScheduledThreadPool(this.simulators.size());
            this.simulators.forEach(simulator -> this.scheduledExecutorService.scheduleAtFixedRate(simulator::tick, 0L, 10L, TimeUnit.MILLISECONDS));
        } else {
            this.scheduledExecutorService = null;
        }
        LOGGER.info("Server started with dimensions {}", Arrays.toString(dimensions));
    }

    public void manualTick() {
        this.simulators.forEach(Simulator::tick);
    }

    public void sendMessageC2S(@Nullable Integer worldIndex, QueueObject queueObject) {
        if (worldIndex == null) {
            this.simulators.forEach(simulator -> simulator.sendMessageC2S(queueObject));
        } else if (worldIndex >= 0 && worldIndex < this.simulators.size()) {
            ((Simulator)this.simulators.get(worldIndex.intValue())).sendMessageC2S(queueObject);
        }
    }

    public void processMessagesS2C(int worldIndex, Consumer<QueueObject> callback) {
        if (worldIndex >= 0 && worldIndex < this.simulators.size()) {
            ((Simulator)this.simulators.get(worldIndex)).processMessagesS2C(callback);
        }
    }

    public void save() {
        this.simulators.forEach(Simulator::save);
    }

    public void stop() {
        LOGGER.info("Stopping...");
        if (this.webserver != null) {
            this.webserver.stop();
        }
        if (this.scheduledExecutorService != null) {
            this.scheduledExecutorService.shutdown();
            Utilities.awaitTermination(this.scheduledExecutorService);
        }
        LOGGER.info("Starting full save...");
        this.simulators.forEach(Simulator::stop);
        LOGGER.info("Stopped");
    }

    /*
     * Enabled aggressive block sorting
     * Enabled unnecessary exception pruning
     * Enabled aggressive exception aggregation
     */
    private void readConsoleInput() {
        try {
            block16: while (true) {
                String[] input = new BufferedReader(new InputStreamReader(System.in)).readLine().trim().toLowerCase(Locale.ENGLISH).replaceAll("[^a-z ]", "").split(" ");
                switch (input[0]) {
                    case "exit": 
                    case "stop": 
                    case "quit": {
                        this.stop();
                        return;
                    }
                    case "save": 
                    case "save-all": {
                        this.save();
                        continue block16;
                    }
                    case "generate": 
                    case "regenerate": {
                        StringBuilder generateKey = new StringBuilder();
                        for (int i = 1; i < input.length; ++i) {
                            generateKey.append(input[i]).append(" ");
                        }
                        this.simulators.forEach(simulator -> Depot.generateDepotsByName(simulator, generateKey.toString()));
                        continue block16;
                    }
                }
                LOGGER.info("Unknown command \"{}\"", input[0]);
            }
        }
        catch (Exception e) {
            LOGGER.error("", (Throwable)e);
            this.stop();
            return;
        }
    }

    private static void printHelp() {
        LOGGER.info("Usage:");
        LOGGER.info("java -jar Transport-Simulation-Core.jar <rootPath> <webserverPort> <useThreadedSimulation> <useThreadedFileLoading> <dimensions...>");
    }

    static {
        LOGGER = LogManager.getLogger((String)"TransportSimulationCore");
    }

    private static class MainWebServlet
    extends WebServlet {
        public MainWebServlet(Function<String, String> contentProvider, String expectedPath) {
            super(contentProvider, expectedPath);
        }
    }
}

