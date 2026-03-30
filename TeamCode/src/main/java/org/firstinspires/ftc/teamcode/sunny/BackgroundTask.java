package org.firstinspires.ftc.teamcode.sunny;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/*
    This class is will run a task (Runnable) at a constant rate
 */

public class BackgroundTask {

    // The CPU scheduler to execute a task after a certain time
    private final ScheduledExecutorService scheduler;

    // The task that will be run at a constant rate
    private final Runnable task;

    // The interval between task executions
    private final long interval;

    // The the time unit for interval ()
    private final TimeUnit unit;


    //-- Constructor --//
    public BackgroundTask(Runnable task, long interval, TimeUnit unit) {
	   this.scheduler = Executors.newScheduledThreadPool(1); // Use a thread pool
	   this.task = task;
	   this.interval = interval;
	   this.unit = unit;
    }

    // Start the background task
    public void start() {
	   scheduler.scheduleWithFixedDelay(task, 0, interval, unit);
    }

    // Shutdown the background task
    public void stop() {
	   scheduler.shutdown();
    }

}