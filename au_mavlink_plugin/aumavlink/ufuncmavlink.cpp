/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                      *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

// Based on aulaserbox and aukinect

#include <cstdlib>
#include <ulms4/ufunclaserbase.h>
#include <ugen4/udatabase.h>
#include <ugen4/upolygon.h>
#include <urob4/uresposehist.h>

#include "serial_port.h"
#include "autopilot_interface.h"

#include <pthread.h>

/* TODO is it ok to have global vars here? */
Autopilot_Interface *autopilot_interface_quit;
Serial_Port *serial_port_quit;
void quit_handler( int sig );

/*
class xxx // TODO the autopilot class
{
public:
	//vars
	
	/// create global variables
	void createGlobalVariables(UVarPool * vp)
	{
		
	}

private:
	//vars
	
public:
	//funcs
	
private:
	//funcs
	
};
*/


class UFuncMavlink : public UFuncLaserBase // TODO change superclass?
{
public:
	
	
private:
	
	
public:
	

	/**
	 * Called by the server after this module is integrated into the server core structure,
	 * i.e. all core services are in place, but no commands are serviced for this module yet.
	 * Create any resources that this modules needs. */
	virtual void createResources()
	{
		// TODO change this. I am not sure what the resources are, because I don't know how variables inside mobotware work
//		myvar.createGlobalVariables((UVarPool *)getVarPool());
		
		
		
		/* TODO is it here that I should write the code that starts everything? Is this my "main"? */


		mavlink_control(/* TODO argc,argv????*/);
		 // ATTENTION! The program will get stuck inside mavlink_control
		
		// Uncomment to use additional thread
		// start thread
//		start_thread();
	}
	/**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
	virtual bool handleCommand(UServerInMsg * msg, void * extra)
	{  // handle a plugin command
		const int MRL = 500;
		char reply[MRL];
		bool ask4help;
		const int MVL = 30;
		char value[MVL];
		ULaserData * data;
		//
		// check for parameters - one parameter is tested for - 'help'
		ask4help = msg->tag.getAttValue("help", value, MVL);
		if (ask4help)
		{ // create the reply in XML-like (html - like) format
			sendHelpStart(msg, "inbox");
			sendText("--- available INBOX options\n");
			sendText("device=port baudrate=B");
			sendText("                add name of connection port of device\n");
			sendHelpDone();
		}
		else
		{ // do some action and send a reply
			
			
			/* TODO I don't think I need to answer to commands, but here's where I should do it if I need */
			
			// If I receive a command from msg
			/*
			 int polySys = -2;
			 msg->tag.getAttInteger("poly", &polySys, 0);
			 */
			// If I receive a command from msg and want to do an action
			
			/* TODO get values from command line*/
			
			// test add option
			if (msg->tag.getAttValue("device", value, MVL))
			{ // get all needed parameters
				double x1, x2, y1, y2;
				bool gotX1, gotX2, gotY1, gotY2;
				gotX1 = msg->tag.getAttDouble("x1", &x1);
				gotX2 = msg->tag.getAttDouble("x2", &x2);
				gotY1 = msg->tag.getAttDouble("y1", &y1);
				gotY2 = msg->tag.getAttDouble("y2", &y2);
				if (gotX1 and gotX2 and gotY1 and gotY2)
				{
					//dostuff
				}
				else
				{
					if (not silent)
					 sendInfo("failed to add box, not enough parameters");
				}
			}
		}
		// return true if the function is handled with a positive result
		return true;
	}
	
	/**
	 * set resource when new resource is loaded */
	virtual bool setResource(UResBase * resource, bool remove)
	{
		/* TODO change this function to my needs */

		}
		result &= UFuncLaserBase::setResource(resource, remove);
		return result;
	}
	/**
	 Run thread */
	void run();
	


	
private:
	/// thread runnung flag
	bool threadRunning;
	/// stop thread flag
	bool threadStop;
	/**
	 Thread handle for frame read thread. */
	pthread_t threadHandle;


private:
	/* TODO do I send vars here? */
	void sendToPolyPlugin()
	{ }

	int mavlink_control(int argc, char **argv);
	int top (int argc, char **argv);
	void stop_mavlink();

	/// start thread
	bool start_thread();
	// stop thread
	void stop_thread();



};



int UFuncMavlink::mavlink_control(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}
	
	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}
}

int UFuncMavlink::top (int argc, char **argv)
{
	
	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------
	
	// Default input arguments
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 921600;
	
	/* TODO should get the parameters from the handlecommand function */
//	parse_commandline(argc, argv, uart_name, baudrate);
	
	
	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------
	
	/*
	 * Instantiate a serial port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock.
	 *
	 */
	Serial_Port serial_port(uart_name, baudrate);
	
	
	/*
	 * Instantiate an autopilot interface object
	 *
	 * This starts two threads for read and write over MAVlink. The read thread
	 * listens for any MAVlink message and pushes it to the current_messages
	 * attribute.  The write thread at the moment only streams a position target
	 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
	 * is changed by using the method update_setpoint().  Sending these messages
	 * are only half the requirement to get response from the autopilot, a signal
	 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
	 * method.  Signal the exit of this mode with disable_offboard_control().  It's
	 * important that one way or another this program signals offboard mode exit,
	 * otherwise the vehicle will go into failsafe.
	 *
	 */
	Autopilot_Interface autopilot_interface(&serial_port);
	
	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	
	signal(SIGINT,quit_handler); // TODO don't know if I need this. If I don't, delete the quit_handler function
	
	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	serial_port.start();
	autopilot_interface.start();
	
	
	// TODO Reading thread has started with autopilot_interface.start();
	
	// Information is accessible, e.g.:
	// autopilot_interface.current_messages.highres_imu.xacc
	// autopilot_interface.current_messages.highres_imu.time_usec
	// autopilot_interface.current_messages.time_stamps.highres_imu
	
	
	return 0;
	
}



void UFuncMavlink::stop_mavlink()
{
	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------
	
	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	autopilot_interface.stop();
	serial_port.stop();
	
	return 0;

}



/* TODO should I delete this? */
// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");
	
	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}
	
	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}
	
	// end program here
	exit(0);
	
}










// ----------------------------------------------------------------------------
// Thread stuff

///////////////////////////////////////////////////

void * startUFuncMavlinkThread(void * obj)
{ // call the hadling function in provided object
	UFuncMavlink * ce = (UFuncMavlink *)obj; /* TODO Change */
	ce->run();
	pthread_exit((void*)NULL); /* TODO use pthread_join() */
	return NULL;
}

///////////////////////////////////////////////////

bool UFuncMavlink::start_thread()
{
	int err = 0;
	
	if (not threadRunning)
	{
		threadStop = false;
		// create socket server thread
		err = (pthread_create(&threadHandle, NULL,
							  &startUFuncMavlinkThread, (void *)this) == 0);
	}
	return err;
}

///////////////////////////////////////////////////

void UFuncMavlink::stop_thread()
{
	if (threadRunning and not threadStop)
	{ // stop and join thread
		threadStop = true;
		pthread_join(threadHandle, NULL);
	}
}


void UFuncMavlink::run()
{
	
	if (threadRunning)
		// prevent nested calls;
		return;
	threadRunning = true;
	
	while (not threadStop)
	{
		// TODO Do stuff in the thread
		
		
		usleep(1000);
		
		
	}
	threadRunning = false;
}




/**
 * Destructor */
UFuncPX4Comm::-UFuncPX4Comm()
{
	stop_mavlink();
	stop_thread();
}

#ifdef LIBRARY_OPEN_NEEDED

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
	return new UFuncMavlink();
}
#endif


/* An additional thread is implemented but not being used in the code.
 To use it, uncomment start_thread() in createResources() */


