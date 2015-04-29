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

// Based on aulaserbox

#include <cstdlib>
#include <ulms4/ufunclaserbase.h>
#include <ugen4/udatabase.h>
#include <ugen4/upolygon.h>
#include <urob4/uresposehist.h>

#include <pthread.h>

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/* TODO Make my classes if I need them or delete this*/
class UNamedBox
{
public:
	
	
	
	/*Class fields*/
	
  /// laser scanner pose
  UPosRot * laserPose;
  /// parent structure for global variables
  UVarPool * parentStruct;
  /// name of structure
  UVariable * varName;

public:
	/* TODO make constructor if needed*/
	/// constructor

	
	/* TODO make Global Vars*/

  /// create global variables
  void createGlobalVariables(UVarPool * vp)
  {
    parentStruct = vp;
    varName = vp->addVarA("name", "", "s", "(r) Name of the box");
    varBoxLimits = vp->addVarA("boxLimits", "0 0 0 0", "d", "(r) box limits [minX, minY, maxX, maxY] in robot coordinates");
    varDetections = vp->addVar("detectCnt", 0.0, "d", "(r) detection count in this box");
    varDetectMinMaxRob = vp->addVarA("detectMinMaxRob", "0 0 0 0", "d", "(r) detections limits [minX, minY, maxX, maxY] in robot coordinates");
    varDetectMinMaxCoo = vp->addVarA("detectMinMaxCoo", "0 0 0 0", "d", "(r) detections limits [minX, minY, maxX, maxY] in selected coordinate system");
    varCooSys = vp->addVar("cooSys", 0.0, "d", "(r/w) Selected coordinate system: 0=odometry, 1=GPS (UTM), 2=Map");
  }
	
	/* Put functions here if I need class functions */
};

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/**
 * Laserscanner function to demonstrate
 * simple laser scanner data handling and analysis
 * @author Christian Andersen
*/
class UFuncPX4Comm : public UFuncLaserBase
{
public:
	
	// Class variables
  // TODO declare class object
	MYCLASS myvar;

public:
  /**
  Constructor */
  UFuncPX4Comm()
  { // set the command (or commands) handled by this plugin
	  setCommand("px4comm", "px4comm"); // TODO change?
	  threadRunning = false;
	  threadStop = false;
  }
  /**
   * Create any resources that this modules needs
   * This method is called after the resource is registred by the server. */
  virtual void createResources()
  {
	  // TODO change this, change myvar
    myvar.createGlobalVariables((UVarPool *)getVarPool());
	  
	  
	  // start read thread
	  start();
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
      sendText("add=name x1=A y1=B x2=C y2=D");
      sendText("                add named box with limits A,B,C,D in robot coordinates\n");
      sendText("detect          Detect for objects in box\n");
      sendText("poly=S          make also result polygons in -1=none, 0=odo (default), 1=UPM, 2=Map coordinates\n");
      sendText("silent          send no (less) reply\n");
      sendText("fake=F          Fake some data 1=random, 2-4 a fake corridor\n");
      sendText("device=N        Laser device to use (see: SCANGET help)\n");
      sendText("see also: SCANGET and SCANSET\n");
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
		/*
      // test add option
      if (msg->tag.getAttValue("add", value, MVL))
      { // get all needed parameters
        double x1, x2, y1, y2;
        bool gotX1, gotX2, gotY1, gotY2;
        gotX1 = msg->tag.getAttDouble("x1", &x1);
        gotX2 = msg->tag.getAttDouble("x2", &x2);
        gotY1 = msg->tag.getAttDouble("y1", &y1);
        gotY2 = msg->tag.getAttDouble("y2", &y2);
        if (gotX1 and gotX2 and gotY1 and gotY2)
        {
          UNamedBox * bx;
          bx = boxes.addBox(value, x1, y1, x2, y2);
          if (not silent)
          {
            if (bx != NULL)
              sendInfo("box added");
            else
              sendWarning("failed to add box");
          }
          if (polySys != -1 and bx != NULL)
          {
            snprintf(reply, MRL, "%s_Box", bx->varName->getString());
            sendToPolyPlugin(reply, &bx->cornerPolygon, 0);
          }
        }
        else
        {
          if (not silent)
            sendInfo("failed to add box, not enough parameters");
        }
      }
		 */
    }
    // return true if the function is handled with a positive result
    return true;
  }

  /**
   * set resource when new resource is loaded */
	virtual bool setResource(UResBase * resource, bool remove)
	{
		/* TODO change this function to my needs */
	  bool result = false;
	  //
	  if (resource->isA(UResPoseHist::getOdoPoseID()))
	  {
		result = true;
		if (remove)
		  boxes.phOdo = NULL;
		else if (boxes.phOdo != resource)
		  boxes.phOdo = (UResPoseHist *)resource;
		else
		  // not used
		  result = false;
	  }
	  else if (resource->isA(UResPoseHist::getMapPoseID()))
	  {
		result = true;
		if (remove)
		  boxes.phMap = NULL;
		else if (boxes.phMap != resource)
		  boxes.phMap = (UResPoseHist *)resource;
		else
		  result = false;
	  }
	  else if (resource->isA(UResPoseHist::getUtmPoseID()))
	  {
		result = true;
		if (remove)
		  boxes.phUtm = NULL;
		else if (boxes.phMap != resource)
		  boxes.phUtm = (UResPoseHist *)resource;
		else
		  // not used
		  result = false;
	  }
	  result &= UFuncLaserBase::setResource(resource, remove);
	  return result;
	}
	/**
  Run receive thread */
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
	/**
	 * */

	/* TODO do I send vars here? */
	void sendToPolyPlugin()
	{ }

	/// start read thread
	bool start();
	// stop read thread
	void stop();


	

};



// ----------------------------------------------------------------------------
// Thread stuff

///////////////////////////////////////////////////

void * startUFuncPX4CommClientThread(void * obj)
{ // call the hadling function in provided object
	UFuncPX4Comm * ce = (UFuncPX4Comm *)obj; /* TODO Change */
	ce->run();
	pthread_exit((void*)NULL); /* TODO use pthread_join() */
	return NULL;
}

///////////////////////////////////////////////////

bool UFuncPX4Comm::start()
{
	int err = 0;

	if (not threadRunning)
	{
		threadStop = false;
		// create socket server thread
		err = (pthread_create(&threadHandle, NULL,
							  &startUFuncPX4CommClientThread, (void *)this) == 0); /* why (void *)? */
	}
	return err;
}

///////////////////////////////////////////////////

void UFuncPX4Comm::stop()
{
	if (threadRunning and not threadStop)
	{ // stop and join thread
		threadStop = true;
		pthread_join(threadHandle, NULL);
	}
}


void UFuncPX4Comm::run()
{

	if (threadRunning)
		// prevent nested calls;
		return;
	threadRunning = true;

	int sockfd, portno, n;
	struct sockaddr_in serv_addr;
	struct hostent *server;
	portno = 11000;

	while (not threadStop)
	{
		


		usleep(1000);
		
		
	}
	threadRunning = false;
}











/**
 * Destructor */
UFuncPX4Comm::-UFuncPX4Comm()
{
	stop();
}

#ifdef LIBRARY_OPEN_NEEDED

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  return new UFuncPX4Comm();
}
#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////






/* My program needs to run: */


int
main(int argc, char **argv)
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



// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{
	
	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------
	
	// Default input arguments
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 921600;
	
	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);
	
	
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
	
	signal(SIGINT,quit_handler); // don't know if I need this. If I don't, delete the quit_handler function
	
	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	serial_port.start();
	autopilot_interface.start();
	
	
	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------
	
	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	autopilot_interface.stop();
	serial_port.stop();
	
	
	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------
	
	// woot!
	return 0;
	
}















// ------------------------------------------------------------------------------
/* Helping functions */


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





// TODO put function parse_commandline into the plugin the way the other plugins do




