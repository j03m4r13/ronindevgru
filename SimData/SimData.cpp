
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include "XPLMMenus.h"
#include <thread>
#include <atomic>
#include "serial.h"
#include <mutex>
#include <vector>

#define MANUAL_PILOT 1
#define MANUAL_PILOT_2 0
#define AUTO_PILOT 2

 // Atomic flag to control thread execution
std::atomic<bool> keepRunning(true), keepRunningForId(true);
std::mutex mtx;  // Mutex to protect shared variable
/* We keep our data ref globally since only one is used for the whole plugin. */
static XPLMDataRef		gDataRef = NULL;
static std::string trimWheelValInStr;
float prevTrimDown = 0.0f;
float currTrimDown = 0.0f;
int prevTrimUp = 0;
int currTrimUp = 0;
int prevAutoPilotMode = 0;
bool hasSentZeroDown = false;
bool hasSentZeroUp = false;
static void	MyMenuHandlerCallback(
                                   void *               inMenuRef,    
                                   void *               inItemRef);    
// Serial port object
serial::Serial arduinoSerial;
serial::Serial arduino;
std::string thePort;
bool foundArduino = false;
bool arduinoConnected = false;
HANDLE hSerial;
DCB dcbSerialParams = { 0 };
COMMTIMEOUTS timeouts = { 0 };



static float	MyFlightLoopCallback(
	float                inElapsedSinceLastCall,
	float                inElapsedTimeSinceLastFlightLoop,
	int                  inCounter,
	void* inRefcon);


std::vector<std::string> list_ports() {
	std::vector<serial::PortInfo> devices_found = serial::list_ports();
	std::vector<std::string> port_names;

	for (serial::PortInfo device : devices_found) {
		port_names.push_back(device.port);
	}

	return port_names;
}

std::string enumerate_ports()
{
	std::vector<serial::PortInfo> devices_found = serial::list_ports();

	std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while (iter != devices_found.end())
	{
		serial::PortInfo device = *iter++;
		std::ostringstream oss;
		oss << "PORTS: " << device.port.c_str() << ": " << device.description.c_str() << " : " << device.hardware_id.c_str()  << std::endl;		
		std::string rxData = oss.str();
		XPLMDebugString(rxData.c_str());
		if (device.hardware_id.find("VID_1A86&PID_7523&REV_0264") != std::string::npos ||
			device.hardware_id.find("VID_2310&PID_8008") != std::string::npos ||
			device.hardware_id.find("VID_2341&PID_8037") != std::string::npos ||
			device.hardware_id.find("VID_2341&PID_8036") != std::string::npos ||
			device.hardware_id.find("VID_2341&PID_8037") != std::string::npos) {

			
			oss << "Found Arduino Micro: " << device.port.c_str() << std::endl;
			XPLMDebugString(oss.str().c_str());
			return device.port;
		}

	}
	return "";
}

void serialIdPollingThread() {
	while (keepRunningForId.load()) {


		// Sleep or wait a bit to avoid overwhelming the serial communication
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	arduino.close();
}

std::string GetAircraftModel() {
	// Find the DataRef for the aircraft model name
	XPLMDataRef modelRef = XPLMFindDataRef("sim/aircraft/view/acf_descrip");
	if (modelRef == nullptr) {
		return "Unknown Model"; // DataRef not found or error
	}

	// Allocate a buffer to hold the model name (X-Plane model names are typically short)
	char modelName[256]; // Adjust buffer size if needed
	int maxLen = sizeof(modelName);

	// Retrieve the model name string from the DataRef
	XPLMGetDatab(modelRef, modelName, 0, maxLen);

	// Return as a C++ string
	return std::string(modelName);
}

// Thread function for polling and sending data via serial
void serialPollingThread() {
	while (keepRunning.load()) {
		//if (!arduinoConnected) continue;
		// sim/cockpit/autopilot/autopilot_mode = 2 else 1
		try {
			gDataRef = XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode");
			if (gDataRef != NULL)
			{
				int autoPilotMode = XPLMGetDatai(gDataRef);

				if (arduinoSerial.isOpen() && autoPilotMode != prevAutoPilotMode && autoPilotMode == MANUAL_PILOT || autoPilotMode == MANUAL_PILOT_2) {
					std::ostringstream oss;
					oss << "M" << std::endl;
					std::string txData = oss.str();
					arduinoSerial.write(txData.c_str());

				}
				else if (arduinoSerial.isOpen() && autoPilotMode != prevAutoPilotMode && autoPilotMode == AUTO_PILOT) {
					std::ostringstream oss;
					oss << "A" << std::endl;
					std::string txData = oss.str();
					arduinoSerial.write(txData.c_str());
				}
				else {
					std::cerr << "Failed to send data to COM port" << std::endl;
				}
				prevAutoPilotMode = autoPilotMode;
			}
			else {
				std::cerr << "Failed to send data to COM port" << std::endl;
			}
			if (arduinoSerial.isOpen()) {
				if (arduinoSerial.available()) {
					std::string receivedData = arduinoSerial.read(arduinoSerial.available());
					int receivedEncoderDirF = std::stof(receivedData);
					std::ostringstream oss;
					oss << "Received: " << receivedData << std::endl;
					std::string rxData = oss.str();
					//XPLMDebugString(rxData.c_str());
					gDataRef = XPLMFindDataRef("sim/cockpit2/controls/elevator_trim");
					if (gDataRef != NULL)
					{
						std::ostringstream oss3;
						float trimWheelValf = XPLMGetDataf(gDataRef);
						if (receivedEncoderDirF == 1) {
							trimWheelValf += 0.002;
							if (trimWheelValf > 1.0) {
								trimWheelValf = 1.0;
							}
						}
						else if (receivedEncoderDirF == -1) {
							trimWheelValf -= 0.002;
							if (trimWheelValf < -1.0) {
								trimWheelValf = -1.0;
							}

						}

						XPLMSetDataf(gDataRef, trimWheelValf);

					}
				}
			}
			else {
				XPLMDebugString("Serial communication is CLOSED");
			}


			std::ostringstream oss;
			std::ostringstream oss2;

			gDataRef = XPLMFindDataRef("sim/cockpit2/controls/elevator_trim");
			if (gDataRef != NULL)
			{
				currTrimDown = XPLMGetDataf(gDataRef);
				if (currTrimDown != prevTrimDown) {
					hasSentZeroDown = false;

					if (arduinoSerial.isOpen()) {
						float trimWheelValf = XPLMGetDataf(gDataRef);
						oss << trimWheelValf;
						std::string trimWheelValInStr = oss.str();
						trimWheelValInStr.append("\n");
						arduinoSerial.write(trimWheelValInStr.c_str());

					}
					else {
						std::cerr << "Failed to send data to COM port" << std::endl;
					}
					prevTrimDown = currTrimDown;
				}


			}


			// Sleep or wait a bit to avoid overwhelming the serial communication
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		} 
		catch (serial::IOException& e) {
			std::ostringstream oss;
			oss << "Error opening serial port: " << e.what() << std::endl;
			XPLMDebugString(oss.str().c_str());
		}

	}

	std::ostringstream oss;
	oss << "Serial communication thread exiting... "  << std::endl;

	XPLMDebugString(oss.str().c_str());
}

// Function to initialize the serial port
bool initSerialPort(const std::string& portName, uint32_t baudrate) {
	try {
		arduinoSerial.setPort(portName);
		arduinoSerial.setBaudrate(baudrate);

		serial::Timeout timeout = serial::Timeout::simpleTimeout(250);
		arduinoSerial.setTimeout(timeout);
		arduinoSerial.open();

		if (arduinoSerial.isOpen()) {
			arduinoSerial.setDTR(true);
			arduinoSerial.setRTS(false);
			std::this_thread::sleep_for(std::chrono::milliseconds(3000));
			std::ostringstream oss;
			oss << "[ASpec] Set DTR AND RTS successful and Port INIT Opened: " << portName << std::endl;

			XPLMDebugString(oss.str().c_str());
			float trimWheelValf = 0.07;
			oss << trimWheelValf;
			std::string trimWheelValInStr = oss.str();
			trimWheelValInStr.append("\n");
			arduinoSerial.write(trimWheelValInStr.c_str());
			return true;
		}
	}
	catch (serial::IOException& e) {
		std::cerr << "Error opening serial port: " << e.what() << std::endl;
		XPLMDebugString("Port ERROR: PORT ");
		return false;
	}
	return false;
}



PLUGIN_API int XPluginStart(
						char *		outName,
						char *		outSig,
						char *		outDesc)
{

		

	XPLMMenuID	myMenu;
	int			mySubMenuItem;
	std::string comPort = enumerate_ports();  // Change this to the detected COM port number
	prevAutoPilotMode = 0;
	//	// Initialize the serial port
	if (comPort.empty()) {
		return 0; // Failed to find Arduino Micro, do not start plugin
	}
	
	if (!initSerialPort(comPort, 115200)) {
		return 0; // Failed to initialize serial, do not start plugin
	}
	arduinoConnected = true;
	
	if (arduinoSerial.isOpen()) {
		std::ostringstream oss;
		oss << "M" << std::endl;
		std::string txData = oss.str();
		arduinoSerial.write(txData.c_str()); // Send Manual Trigger // Release Stepper
	}

	/* Provide our plugin's profile to the plugin system. */
	strcpy(outName, "ASpec");
	strcpy(outSig, "com.aspec.twv1");
	strcpy(outDesc, "A plugin for trim wheel module.");

	/* First we put a new menu item into the plugin menu.
	 * This menu item will contain a submenu for us. */
	mySubMenuItem = XPLMAppendMenuItem(
						XPLMFindPluginsMenu(),	/* Put in plugins menu */
						"ASpec",				/* Item Title */
						0,						/* Item Ref */
						1);						/* Force English */
	
	/* Now create a submenu attached to our menu item. */
	myMenu = XPLMCreateMenu(
						"Trim Wheel V1", 
						XPLMFindPluginsMenu(), 
						mySubMenuItem, 			/* Menu Item to attach to. */
						MyMenuHandlerCallback,	/* The handler */
						0);						/* Handler Ref */
	/* Append a few menu items to our submenu.  We will use the refcon to
 * store the amount we want to change the radio by. */
	//XPLMAppendMenuItem(
	//	myMenu,
	//	"SEND Message",
	//	(void*)-1000,
	//	1);
	//XPLMAppendMenuItem(
	//	myMenu,
	//	"Receive Message",
	//	(void*)+1000,
	//	1);

		// Start the serial communication thread
	std::thread(serialPollingThread).detach();  // Detach the thread so it runs independently

	gDataRef = XPLMFindDataRef("sim/cockpit2/controls/elevator_trim");


	/* Only return that we initialized correctly if we found the data ref. */
	return (gDataRef != NULL) ? 1 : 0;
}


PLUGIN_API void	XPluginStop(void)
{

	if (arduinoSerial.isOpen()) {
		std::ostringstream oss;
		oss << "M" << std::endl;
		std::string txData = oss.str();
		arduinoSerial.write(txData.c_str()); // Send Manual Trigger
	}
    keepRunning.store(false);  // Signal the thread to stop
	XPLMUnregisterFlightLoopCallback(MyFlightLoopCallback, NULL);
	// Stop the serial communication thread
	if (arduino.isOpen()) {
		arduino.close();
	}
	// Close serial port
	if (arduinoSerial.isOpen()) {
		arduinoSerial.close();
	}
}

PLUGIN_API void XPluginDisable(void)
{
	if (arduinoSerial.isOpen()) {
		std::ostringstream oss;
		oss << "M" << std::endl;
		std::string txData = oss.str();
		arduinoSerial.write(txData.c_str()); // Send Manual Trigger

	}
	keepRunning.store(false);  // Signal the thread to stop
	XPLMUnregisterFlightLoopCallback(MyFlightLoopCallback, NULL);
	//// Stop the serial communication thread

	//// Close serial port
	if (arduinoSerial.isOpen()) {
		arduinoSerial.close();
	}
}

PLUGIN_API int XPluginEnable(void)
{

	if (!arduinoSerial.isOpen()) {
		std::string comPort = enumerate_ports();  // Change this to the detected COM port number

		// Initialize the serial port
		if (comPort.empty()) {
			return 0; // Failed to find Arduino Micro, do not start plugin
		}

		if (!initSerialPort(comPort, 115200)) {
			return 0; // Failed to initialize serial, do not start plugin
		}
		prevAutoPilotMode = 0;
		keepRunning.store(true);
		std::thread(serialPollingThread).detach();
	}

	return 1;
}

PLUGIN_API void XPluginReceiveMessage(
					XPLMPluginID	inFromWho,
					int				inMessage,
					void *			inParam)
{

}


void	MyMenuHandlerCallback(
                                   void *               inMenuRef,    
                                   void *               inItemRef)
{
	/* This is our handler for the menu item.  Our inItemRef is the refcon
	 * we registered in our XPLMAppendMenuItem calls.  It is either +1000 or
	 * -1000 depending on which menu item is picked. */
	if (arduinoSerial.isOpen()) {
		arduinoSerial.write("Test");
	}
}                                   

float	MyFlightLoopCallback(
	float                inElapsedSinceLastCall,
	float                inElapsedTimeSinceLastFlightLoop,
	int                  inCounter,
	void* inRefcon)
{
	/* The actual callback.  First we read the sim's time and the data. */
	float	elapsed = XPLMGetElapsedTime();

	std::ostringstream oss;
	gDataRef = XPLMFindDataRef("sim/cockpit2/controls/elevator_trim");
	if (gDataRef != NULL)
	{
		float trimWheelValf = XPLMGetDataf(gDataRef);

		oss << trimWheelValf;
		std::lock_guard<std::mutex> lock(mtx);  // Lock the mutex		
		trimWheelValInStr = oss.str();
		trimWheelValInStr.append("\n");
		std::string writeData = trimWheelValInStr;

		//arduinoSerial.write(writeData);

	}

	/* Return 1.0 to indicate that we want to be called again in 1 second. */
	return 0.1;
}