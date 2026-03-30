package org.firstinspires.ftc.teamcode.util;

public class ParallelMotion {

    Runnable task1;
    Runnable task2;

    Thread task1Thread;
    Thread task2Thread;

    public ParallelMotion(Runnable task1, Runnable task2){
	   this.task1 = task1;
	   this.task2 = task2;

	   task1Thread = new Thread(task1);
	   task2Thread = new Thread(task2);

    }

    public void start(){
	   task1Thread.start();
	   task2Thread.start();

	   try{
		  task1Thread.join();
		  task2Thread.join();
	   } catch (Exception e){
		  System.out.println("Error" + e.getMessage());
	   }
    }

}
