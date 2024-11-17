
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include "XPLMDisplay.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMPlanes.h"
#include "XPLMGraphics.h" 
#include <thread>
#include <atomic>
#include "serial.h"
#include <mutex>
#include <vector>
#include <cmath>
#define XPLM_MSG_PLANE_LOADED 102
#define TW_PLUGIN 1
#define FS_PLUGIN 2

#define MANUAL_PILOT 1
#define MANUAL_PILOT_2 0
#define AUTO_PILOT 2
#define YOKE_TRIM_MODE 3
#define TBM_TRIM_ON 2
#define TBM_AP_ON 2
#define TBM_AUTOPILOT_MODE_OFF 1
#define TBM_AUTOPILOT_MODE_ON 2

#define TBM_STATE_TRIM_OFF 0
#define TBM_STATE_TRIM_AP_OFF 1
#define TBM_STATE_TRIM_ON 2
#define TBM_STATE_AUTOPILOT_ON 3
#define TBM_STATE_TRIM_ON_MANUAL_PITCH_ON 4

#define FUEL_AUTO 1
#define FUEL_MANUAL 2
// tbm900/actuators/ap/disc 
// sim/cockpit/autopilot/autopilot_mode = 2 else 1
//tbm900/switches/elec/ap_trims OFF = 0, AP OFF = 1, ON = 2
// sim/flight_controls/pitch_trim_down
// sim/flight_controls/pitch_trim_up


//tbm900/switches/fuel/fuel_sel 0.0 = off, r = 1.0, l = 2
//sim/fuel/fuel_selector_lft
//sim/fuel/fuel_selector_rgt
//sim/fuel/fuel_selector_none
//************************************
// FUEL SELECTOR DEFINITIONS
std::mutex dataRefMutex;
std::mutex dataMutex, dataMutexTW;
serial::Serial fuelSelectorSerial;
std::atomic<bool> keepRunningFuelSelector(true);

// SHARED DATA
std::string receivedData;
std::string receivedDataFS;
float prevTrimDown = 0.0f;
float currTrimDown = 0.0f;
/*********************/
static XPLMDataRef		gFuelSelectorDataRef = NULL;
static XPLMDataRef		gFuelModeDataRef = NULL;
float currFuelSelectorPosition = 0.0;
float prevFuelSelectorPosition = 0.0;
int fuelSelectorState = 0;
int prevFuelSelectorState = 0;
XPLMCommandRef fuelLeftCommand;
XPLMCommandRef fuelRightCommand;
XPLMCommandRef fuelOffCommand;

float currentFuelMode = 0.0;
float prevFuelMode = 0.0;


bool fuelLeftActive = false;
bool fuelRightActive = false;
bool fuelOffActive = false;

bool fuelSelectorInitCompleted = false;
//************************************
// TRIM WHEEL DEFINITION
// Atomic flag to control thread execution
std::atomic<bool> keepRunning(true), keepRunningForId(true);
std::atomic<bool> triggerAPDisc{ false };
std::mutex mtx;  // Mutex to protect shared variable
/* We keep our data ref globally since only one is used for the whole plugin. */
static XPLMDataRef		gDataRef = NULL;
static XPLMDataRef		gDataRefTBM = NULL;
static XPLMDataRef		gDataRefTBMPitchUpSwitch = NULL;
static XPLMDataRef		gDataRefTBMPitchDownSwitch = NULL;

XPLMCommandRef pitchTrimUpCommand;
XPLMCommandRef pitchTrimDownCommand;
XPLMCommandRef autopilotDisconnectCommand;
XPLMCommandRef CWSCommand;
XPLMCommandRef PTTCommand;
XPLMCommandRef LeftFuelSelectorCommand;
XPLMCommandRef RightFuelSelectorCommand;
XPLMCommandRef OffFuelSelectorCommand;
XPLMCommandRef ShiftFuelSelectorCommand;

static std::string trimWheelValInStr;

int prevTrimUp = 0;
int currTrimUp = 0;
int prevAutoPilotMode = 0;
int currAutoPilotMode = 0;
int prevAPModeTBM = 0;
int prevState = -1;
int currentTBMTrimState = 0;
int autoPilotModeNormal = 0;

bool hasSentZeroDown = false;
bool hasSentZeroUp = false;

bool pitchTrimUpActive = false;
bool pitchTrimDownActive = false;
bool autoPilotDisconnected = false;
bool triggerAP = false;
bool planeLoadedFully = false;
bool trimWheelActive = false;
bool fuelSelectorActive = false;
bool CWSActive = false;

// MENU GUI
int fsMenuIdx, twMenuIdx;
XPLMMenuID	myMenu;
int aboutMenuItemIndex;
XPLMWindowID aboutWindow = NULL;
static XPLMWindowID	g_window;
void createAboutWindow();
void drawAboutWindow(XPLMWindowID inWindowID, void* inRefcon);
int handleMouseClick(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void* inRefcon);
void closeAboutWindow();
void serialCommsThreadFS();

//*******************************************

static void	MyMenuHandlerCallback(
	void* inMenuRef,
	void* inItemRef);
// Serial port object
serial::Serial arduinoSerial;
serial::Serial arduino;
std::string thePort;
std::string userAircraft;
bool foundArduino = false;
bool arduinoConnected = false;
bool hasLoadedAircraftName = false;
HANDLE hSerial;
DCB dcbSerialParams = { 0 };
COMMTIMEOUTS timeouts = { 0 };



std::vector<std::string> list_ports() {
	std::vector<serial::PortInfo> devices_found = serial::list_ports();
	std::vector<std::string> port_names;

	for (serial::PortInfo device : devices_found) {
		port_names.push_back(device.port);
	}

	return port_names;
}

std::string enumerate_ports(int forPlugin = TW_PLUGIN)
{
	std::vector<serial::PortInfo> devices_found = serial::list_ports();

	std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while (iter != devices_found.end())
	{
		serial::PortInfo device = *iter++;
		std::ostringstream oss;
		//oss << "PORTS: " << device.port.c_str() << ": " << device.description.c_str() << " : " << device.hardware_id.c_str()  << std::endl;		
		//std::string rxData = oss.str();
		//XPLMDebugString(rxData.c_str());
		if (forPlugin == TW_PLUGIN) {
			if (device.hardware_id.find("VID_1A86&PID_7523&REV_0264") != std::string::npos ||
				device.hardware_id.find("VID_2310&PID_8008") != std::string::npos ||
				device.hardware_id.find("VID_2341&PID_8037") != std::string::npos ||
				device.hardware_id.find("VID_2341&PID_8036") != std::string::npos ||
				device.hardware_id.find("VID_2341&PID_8037") != std::string::npos ||
				device.hardware_id.find("VID_4210&PID_6900") != std::string::npos ||
				device.hardware_id.find("VID_2342&PID_3501") != std::string::npos ||
				device.hardware_id.find("VID_4262PID_3500") != std::string::npos) {


				oss << "[RoninDevGru] Found Arduino Micro for the TrimWheel Module: " << device.port.c_str() << std::endl;
				XPLMDebugString(oss.str().c_str());
				return device.port;
			}
		}
		else {

			if (device.hardware_id.find("VID_4262&PID_3501") != std::string::npos ||
				device.hardware_id.find("VID_2342&PID_3502") != std::string::npos) {

				oss << "[RoninDevGru] Found Arduino Micro for the Fuel Selector Module: " << device.port.c_str() << std::endl;
				XPLMDebugString(oss.str().c_str());
				return device.port;
			}
		}

	}
	return "";
}


std::string getFirstAircraftModel() {
	char modelName[256];
	char modelPath[512];

	// Retrieve model name and path for the first aircraft (index 0)
	XPLMGetNthAircraftModel(0, modelName, modelPath);
	std::ostringstream oss;
	oss << modelName;
	std::string aircraft = oss.str();


	return aircraft;
}

bool isInFlight() {

	// DataRefs for simulator state
	XPLMDataRef pausedRef = XPLMFindDataRef("sim/time/paused");
	XPLMDataRef xPosRef = XPLMFindDataRef("sim/flightmodel/position/local_x");
	XPLMDataRef yPosRef = XPLMFindDataRef("sim/flightmodel/position/local_y");
	XPLMDataRef zPosRef = XPLMFindDataRef("sim/flightmodel/position/local_z");

	if (pausedRef && xPosRef && yPosRef && zPosRef) {
		int paused = XPLMGetDatai(pausedRef);
		float xPos = XPLMGetDataf(xPosRef);
		float yPos = XPLMGetDataf(yPosRef);
		float zPos = XPLMGetDataf(zPosRef);

		// If simulator is not paused and aircraft has valid position, assume flight is active
		if (paused == 0 && (xPos != 0.0 || yPos != 0.0 || zPos != 0.0)) {
			return true;
		}
	}
	
	return false;
}

//***** TRIM WHEEL FUNCTIONS **********************************/

void ExecutePitchTrimUp() {
	if (pitchTrimUpCommand) {
		// Begin the command (simulates pressing the button)
		XPLMCommandOnce(pitchTrimUpCommand);

		// Optionally, end the command immediately to simulate a quick press
		//XPLMCommandEnd(pitchTrimUpCommand);
	}
	else {
		// Handle error: command not found
	}
}
void ExecutePitchTrimDown() {
	if (pitchTrimDownCommand) {
		// Begin the command (simulates pressing the button)
		XPLMCommandOnce(pitchTrimDownCommand);

		//// Optionally, end the command immediately to simulate a quick press
		//XPLMCommandEnd(pitchTrimDownCommand);
	}
	else {
		// Handle error: command not found
	}
}

void DisconnectAutopilot() {
	if (autopilotDisconnectCommand != nullptr) {
		// Trigger the disconnect command
		XPLMCommandOnce(autopilotDisconnectCommand);
	}
	else {
		// Handle error: command not found
	}
}

void DisconnectAutopilotUsingKeyID() {
	// Sends a command keystroke to disarm the autopilot
	XPLMCommandKeyStroke(xplm_key_elvtrimU);
}

void CommandThread() {
	// Wait or perform calculations here

	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Example delay

	// Set flag for main thread to pick up
	std::lock_guard<std::mutex> lock(dataMutex);
	triggerAPDisc.store(true);
}

void MyMessageHandler(
	XPLMPluginID inFromWho,
	int inMessage,
	void* inParam)
{
	if (inMessage == XPLM_MSG_PLANE_LOADED) {
		// This message indicates that the aircraft is fully loaded
		// The plugin can be considered fully loaded at this point
		planeLoadedFully = true;
	}
}


float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon) {
	std::lock_guard<std::mutex> lock(dataMutex);
	if (triggerAPDisc.load()) {
		// Execute pitch trim up command
		if (autopilotDisconnectCommand != nullptr) {
			// Trigger the disconnect command
			XPLMCommandBegin(autopilotDisconnectCommand);
			XPLMCommandEnd(autopilotDisconnectCommand);
		}
		else {
			// Handle error: command not found
		}
		// Reset the flag
		triggerAPDisc.store(false);
	}
	return -1.0f; // Return -1 to run every frame
}


// Command handler function
int PitchTrimUpCommandHandler(XPLMCommandRef command, XPLMCommandPhase phase, void* refcon) {
	std::lock_guard<std::mutex> lock(dataMutex);
	if (phase == xplm_CommandBegin) {
		// Command has started (button pressed)
		pitchTrimUpActive = true;
	}
	else if (phase == xplm_CommandEnd) {
		// Command has ended (button released)
		pitchTrimUpActive = false;
	}
	return 1; // Return 1 to let other plugins also handle the command
}

// Command handler function
int PitchTrimDownCommandHandler(XPLMCommandRef command, XPLMCommandPhase phase, void* refcon) {
	std::lock_guard<std::mutex> lock(dataMutex);
	if (phase == xplm_CommandBegin) {
		// Command has started (button pressed)
		pitchTrimDownActive = true;
	}
	else if (phase == xplm_CommandEnd) {
		// Command has ended (button released)
		pitchTrimDownActive = false;
	}
	return 1; // Return 1 to let other plugins also handle the command
}


// Command handler function
int CWSCommandHndler(XPLMCommandRef command, XPLMCommandPhase phase, void* refcon) {
	std::lock_guard<std::mutex> lock(dataMutex);
	if (phase == xplm_CommandBegin) {
		// Command has started (button pressed)
		CWSActive = true;
	}
	else if (phase == xplm_CommandEnd) {
		// Command has ended (button released)
		CWSActive = false;
	}
	return 1; // Return 1 to let other plugins also handle the command
}
// ******************** END OF TRIMWHEEL FUNCTIONS *********************//



// ******************** FUEL SELECTOR FUNCTIONS ************************//

// Command handler function
int fuelLeftCommandHandler(XPLMCommandRef command, XPLMCommandPhase phase, void* refcon) {
	if (phase == xplm_CommandBegin) {
		// Command has started (button pressed)
		fuelLeftActive = true;
	}
	else if (phase == xplm_CommandEnd) {
		// Command has ended (button released)
		fuelLeftActive = false;
	}
	return 1; // Return 1 to let other plugins also handle the command
}

// Command handler function
int fuelRightCommandHandler(XPLMCommandRef command, XPLMCommandPhase phase, void* refcon) {
	if (phase == xplm_CommandBegin) {
		// Command has started (button pressed)
		fuelRightActive = true;
	}
	else if (phase == xplm_CommandEnd) {
		// Command has ended (button released)
		fuelRightActive = false;
	}
	return 1; // Return 1 to let other plugins also handle the command
}

// Command handler function
int fuelOffCommandHandler(XPLMCommandRef command, XPLMCommandPhase phase, void* refcon) {
	if (phase == xplm_CommandBegin) {
		// Command has started (button pressed)
		fuelOffActive = true;
	}
	else if (phase == xplm_CommandEnd) {
		// Command has ended (button released)
		fuelOffActive = false;
	}
	return 1; // Return 1 to let other plugins also handle the command
}



float MainFlightLoop(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon) {

		std::lock_guard<std::mutex> lock(dataMutex);
		if (!hasLoadedAircraftName) {

			if (userAircraft.empty()) {
				userAircraft = getFirstAircraftModel();
			}

			hasLoadedAircraftName = true;

			pitchTrimUpCommand = XPLMFindCommand("sim/flight_controls/pitch_trim_up");
			pitchTrimDownCommand = XPLMFindCommand("sim/flight_controls/pitch_trim_down");
			CWSCommand = XPLMFindCommand("sim/autopilot/control_wheel_steer");
			PTTCommand = XPLMFindCommand("sim/operation/contact_atc");
			//sim/fuel/fuel_selector_lft
			//sim/fuel/fuel_selector_rgt
			//sim/fuel/fuel_selector_none


			autopilotDisconnectCommand = XPLMFindCommand("sim/autopilot/disconnect");
			gDataRef = XPLMFindDataRef("sim/flightmodel/controls/elv_trim");



			XPLMCommandRef pitchTrimUpCommand = XPLMFindCommand("sim/flight_controls/pitch_trim_up");
			if (pitchTrimUpCommand != nullptr) {
				XPLMRegisterCommandHandler(pitchTrimUpCommand, PitchTrimUpCommandHandler, 1, nullptr);
			}

			XPLMCommandRef pitchTrimDownCommand = XPLMFindCommand("sim/flight_controls/pitch_trim_down");
			if (pitchTrimDownCommand != nullptr) {
				XPLMRegisterCommandHandler(pitchTrimDownCommand, PitchTrimDownCommandHandler, 1, nullptr);
			}


			pitchTrimUpCommand = XPLMFindCommand("sim/flight_controls/pitch_trim_up");

			XPLMRegisterFlightLoopCallback(FlightLoopCallback, -1.0f, nullptr);
			keepRunning.store(true);

			std::thread(serialCommsThreadFS).detach();
			fuelSelectorActive = true;
			trimWheelActive = true;
			prevState = -1;
		}


		if (trimWheelActive) {
			try {
				if (hasLoadedAircraftName && userAircraft == "TBM900.acf") {

					gDataRefTBM = XPLMFindDataRef("tbm900/switches/elec/ap_trims_raw");
					gDataRef = XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode");

					int tbmTrimMode = XPLMGetDatai(gDataRefTBM);
					int apModeTBM = XPLMGetDatai(gDataRef);
					int currentState = -1;

					if (tbmTrimMode == TBM_STATE_TRIM_OFF) {
						currentTBMTrimState = TBM_STATE_TRIM_OFF; // save current state;
					}
					else if (tbmTrimMode == TBM_STATE_TRIM_ON) {
						currentTBMTrimState = TBM_STATE_TRIM_ON; // save current state;
					}

					if (apModeTBM == TBM_AP_ON) {
						currentTBMTrimState = TBM_STATE_AUTOPILOT_ON; // save current state;
						autoPilotDisconnected = false;
					}

					if (pitchTrimDownActive || pitchTrimUpActive) {
						currentTBMTrimState = TBM_STATE_TRIM_ON_MANUAL_PITCH_ON; // 
					}

				}
				else {

					gDataRef = XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode");
					if (gDataRef != NULL)
					{
						
						currAutoPilotMode = XPLMGetDatai(gDataRef);

						if (pitchTrimDownActive || pitchTrimUpActive) {
							currAutoPilotMode = YOKE_TRIM_MODE;
						}
						
					}
					else {
						std::cerr << "Failed to send data to COM port" << std::endl;
					}

				}

				if (receivedData != "") {
					int receivedEncoderDirF = std::stof(receivedData);
					std::ostringstream oss;
					oss << "Received: " << receivedData << std::endl;
					std::string rxData = oss.str();

					XPLMDataRef gElevTrimDataRef = XPLMFindDataRef("sim/cockpit2/controls/elevator_trim");
					if (gElevTrimDataRef != NULL)
					{
						std::ostringstream oss3;
						float trimWheelValf = XPLMGetDataf(gElevTrimDataRef);
						if (receivedEncoderDirF == 1) {
							trimWheelValf += 0.0025;
							if (trimWheelValf > 1.0) {
								trimWheelValf = 1.0;
							}
							XPLMSetDataf(gElevTrimDataRef, trimWheelValf);
						}
						else if (receivedEncoderDirF == -1) {

							trimWheelValf -= 0.0025;
							if (trimWheelValf < -1.0) {
								trimWheelValf = -1.0;
							}
							XPLMSetDataf(gElevTrimDataRef, trimWheelValf);
						}
						else if (receivedEncoderDirF == 2) { // ap disc
							triggerAPDisc.store(true);

						}
						else if (receivedEncoderDirF == 3) { // CWS Press
							if (CWSCommand != nullptr) {
								XPLMCommandBegin(CWSCommand);
							}
						}
						else if (receivedEncoderDirF == 4) { // CWS Depress
							if (CWSCommand != nullptr) {
								XPLMCommandEnd(CWSCommand);
							}
						}
						else if (receivedEncoderDirF == 5) { // PTT Toggle
							if (PTTCommand != nullptr) {
								XPLMCommandOnce(PTTCommand);
							}
						}

					}


				}

				gDataRef = XPLMFindDataRef("sim/flightmodel/controls/elv_trim");
				if (gDataRef != NULL)
				{
					currTrimDown = XPLMGetDataf(gDataRef);
				}

			}
			catch (serial::IOException& e) {
				std::ostringstream oss;
				oss << "[RoninDevGru] Error opening serial port: " << e.what() << std::endl;
				XPLMDebugString(oss.str().c_str());
			}

		}

		if (fuelSelectorActive) {

			LeftFuelSelectorCommand = XPLMFindCommand("sim/fuel/fuel_selector_lft");
			RightFuelSelectorCommand = XPLMFindCommand("sim/fuel/fuel_selector_rgt");
			OffFuelSelectorCommand = XPLMFindCommand("sim/fuel/fuel_selector_none");
			ShiftFuelSelectorCommand = XPLMFindCommand("tbm900/actuators/fuel/shift");

			gFuelSelectorDataRef = XPLMFindDataRef("tbm900/switches/fuel/fuel_sel");
			gFuelModeDataRef = XPLMFindDataRef("tbm900/switches/fuel/auto_man");

			if (gFuelSelectorDataRef != NULL && gFuelModeDataRef != NULL) {

					if (gFuelSelectorDataRef != NULL) {
						std::ostringstream oss;

						int fuelMode = XPLMGetDatai(gFuelModeDataRef);
				
						currentFuelMode = fuelMode;	
					}
			}

			try {
				if (userAircraft == "TBM900.acf") {
					if (gFuelSelectorDataRef != NULL)
					{
						currFuelSelectorPosition = XPLMGetDataf(gFuelSelectorDataRef);
					}
					if (receivedDataFS != "") {
						int receivedFSDir = std::stof(receivedDataFS);
						std::ostringstream oss;
						oss << "Received: " << receivedFSDir << std::endl;
						XPLMDebugString(oss.str().c_str());
						if (receivedFSDir == 1) { // RIGHT
							if (RightFuelSelectorCommand != nullptr) {
								XPLMCommandOnce(RightFuelSelectorCommand);
							}
						}
						else if (receivedFSDir == 2) { // LEFT
							if (LeftFuelSelectorCommand != nullptr) {
								XPLMCommandOnce(LeftFuelSelectorCommand);
							}

						}
						else if (receivedFSDir == 3) { // OFF
							if (OffFuelSelectorCommand != nullptr) {
								XPLMCommandOnce(OffFuelSelectorCommand);
							}
						}
						else if (receivedFSDir == 4) { // SHIFT
							if (ShiftFuelSelectorCommand != nullptr) {
								XPLMCommandBegin(ShiftFuelSelectorCommand); // only for shift
		
							}
						}
						else if (receivedFSDir == 5) { // SHIFT
							if (ShiftFuelSelectorCommand != nullptr) {
								XPLMCommandEnd(ShiftFuelSelectorCommand);
							}
						}
					}
				}

			}
			catch (serial::IOException& e) {
				std::ostringstream oss;
				oss << "[RoninDevGru] Error opening serial port: " << e.what() << std::endl;
				XPLMDebugString(oss.str().c_str());
			}
		}
	

	return -1.0f; // Return -1 to run every frame
}

void serialCommsThreadFS() {
	while (keepRunning.load()) {
		
		// TRIMWHEEL SERIAL RECEPTION
		//std::ostringstream oss;
		//oss << "[RoninDevGru] ALIVE FROM HERE" << std::endl;
		//XPLMDebugString(oss.str().c_str());
		if (fuelSelectorSerial.isOpen()) {
			std::lock_guard<std::mutex> lock(dataMutex);
			if (fuelSelectorActive) {
				int fuelMode = currentFuelMode;
				if (fuelMode != prevFuelMode) {
					if (fuelMode == 0) {
						std::ostringstream oss;
						oss << "R";
						std::string fuelModeStr = oss.str();
						fuelModeStr.append("\n");
						//XPLMDebugString(fuelModeStr.c_str());
						fuelSelectorSerial.write(fuelModeStr.c_str()); // Send Auto Trigger
						prevFuelSelectorPosition = 1.0;
					}
					else if (fuelMode == 1) {
						std::ostringstream oss;
						oss << "T";
						std::string fuelModeStr = oss.str();
						fuelModeStr.append("\n");
						//XPLMDebugString(fuelModeStr.c_str());
						fuelSelectorSerial.write(fuelModeStr.c_str()); // Send Manual Trigger
					}
					prevFuelMode = fuelMode;
				}

				// FUEL SELECTOR SERIAL RECEPTION
				if (fuelSelectorSerial.available()) {
					receivedDataFS = fuelSelectorSerial.read(fuelSelectorSerial.available());
				}
				else {
					receivedDataFS = "";
				}


				float fuelSelectorPos = currFuelSelectorPosition;
				if (fuelSelectorPos != prevFuelSelectorPosition) {
					std::ostringstream oss;
					oss << fuelSelectorPos;
					std::string fuelSelectorPositionInVal = oss.str();
					fuelSelectorPositionInVal.append("\n");
					//XPLMDebugString(fuelSelectorPositionInVal.c_str());
					fuelSelectorSerial.write(fuelSelectorPositionInVal.c_str()); // Send Manual Trigger
					prevFuelSelectorPosition = fuelSelectorPos;
				}
				
			}


		}


		if (arduinoSerial.isOpen()) {
			std::lock_guard<std::mutex> lock(dataMutex);
			if (trimWheelActive) {
				if (userAircraft == "TBM900.acf") {
					if (currentTBMTrimState != prevState) {
						std::ostringstream oss;
						oss << "[RoninDevGru] TBM TRIM STATE:" << currentTBMTrimState << std::endl;
						//XPLMDebugString(oss.str().c_str());
						if (currentTBMTrimState == TBM_STATE_TRIM_OFF) {
							std::ostringstream oss;
							oss << "N" << std::endl;
							std::string txData = oss.str();
							arduinoSerial.write(txData.c_str());
						}
						else if (currentTBMTrimState == TBM_STATE_TRIM_ON) {
							std::ostringstream oss;
							oss << "M" << std::endl;
							std::string txData = oss.str();
							arduinoSerial.write(txData.c_str());
						}
						else if (currentTBMTrimState == TBM_STATE_AUTOPILOT_ON) {
							std::ostringstream oss;
							oss << "A" << std::endl;
							std::string txData = oss.str();
							arduinoSerial.write(txData.c_str());
						}
						else if (currentTBMTrimState == TBM_STATE_TRIM_ON_MANUAL_PITCH_ON) {
							std::ostringstream oss;
							oss << "O" << std::endl;
							std::string txData = oss.str();
							arduinoSerial.write(txData.c_str());
						}
						prevState = currentTBMTrimState;
					}

				}
				else {
					if (currAutoPilotMode != prevAutoPilotMode && currAutoPilotMode == MANUAL_PILOT || currAutoPilotMode == MANUAL_PILOT_2) {

						//NORMAL NON-TBM STUFF
						std::ostringstream oss;
						oss << "M" << std::endl;
						std::string txData = oss.str();
						arduinoSerial.write(txData.c_str());


					}
					else if (currAutoPilotMode != prevAutoPilotMode && currAutoPilotMode == AUTO_PILOT) {

						//NORMAL NON-TBM STUFF
						std::ostringstream oss;
						oss << "A" << std::endl;
						std::string txData = oss.str();
						arduinoSerial.write(txData.c_str());
						autoPilotDisconnected = false;

					}
					else if (currAutoPilotMode != prevAutoPilotMode && currAutoPilotMode == YOKE_TRIM_MODE) {

						std::ostringstream oss;
						oss << "O" << std::endl;
						std::string txData = oss.str();
						arduinoSerial.write(txData.c_str());

					}
					else {
						std::cerr << "Failed to send data to COM port" << std::endl;
					}
					prevAutoPilotMode = currAutoPilotMode;


				}

				

				// TRIMWHEEL SERIAL RECEPTION
				if (arduinoSerial.available()) {
					receivedData = arduinoSerial.read(arduinoSerial.available());
				}
				else {
					receivedData = "";
				}


				//
				if (currTrimDown != prevTrimDown) {
					std::ostringstream oss;
					float trimWheelValf = XPLMGetDataf(gDataRef);
					oss << trimWheelValf;
					std::string trimWheelValInStr = oss.str();
					trimWheelValInStr.append("\n");
					XPLMDebugString(trimWheelValInStr.c_str());
					arduinoSerial.write(trimWheelValInStr.c_str());
					prevTrimDown = currTrimDown;
				}
			}

		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}


// Function to initialize the serial port
bool initSerialPort(const std::string& portName, uint32_t baudrate, int whichPlugin = TW_PLUGIN) {
	try {
		if (whichPlugin == TW_PLUGIN) {
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
				oss << "[RoninDevGru] Set DTR AND RTS successful for TrimWheel Module using " << portName << std::endl;
				XPLMDebugString(oss.str().c_str());
				float trimWheelValf = 0.07;
				oss << trimWheelValf;
				std::string trimWheelValInStr = oss.str();
				trimWheelValInStr.append("\n");
				arduinoSerial.write(trimWheelValInStr.c_str());
				return true;
			}

		}
		else {
			fuelSelectorSerial.setPort(portName);
			fuelSelectorSerial.setBaudrate(baudrate);

			serial::Timeout timeout = serial::Timeout::simpleTimeout(250);
			fuelSelectorSerial.setTimeout(timeout);
			fuelSelectorSerial.open();

			if (fuelSelectorSerial.isOpen()) {
				fuelSelectorSerial.setDTR(true);
				fuelSelectorSerial.setRTS(false);
				std::this_thread::sleep_for(std::chrono::milliseconds(3000));
				std::ostringstream oss;
				oss << "[RoninDevGru] Set DTR AND RTS successful for FuelSelector Module using " << portName << std::endl;

				XPLMDebugString(oss.str().c_str());
				return true;
			}

		}
	}
	catch (serial::IOException& e) {
		std::cerr << "Error opening serial port: " << e.what() << std::endl;
		std::ostringstream oss;
		oss << "[RoninDevGru] Set DTR AND RTS successful for FuelSelector Module using " << e.what() << std::endl;
		XPLMDebugString("[RoninDevGru] Port ERROR: PORT ");
		return false;
	}
	return false;
}
//
//
//// Thread function for polling and sending data via serial
//void serialCommsThreadTW() {
//	while (keepRunningFuelSelector.load()) {
//		
//		std::lock_guard<std::mutex> lock(dataMutex);
//
//		if (arduinoSerial.isOpen()) {
//			
//			if (currentTBMTrimState != prevState) {
//				std::ostringstream oss;
//				oss << "[RoninDevGru] TBM TRIM STATE:" << currentTBMTrimState << std::endl;
//				//XPLMDebugString(oss.str().c_str());
//				if (currentTBMTrimState == TBM_STATE_TRIM_OFF) {
//					std::ostringstream oss;
//					oss << "N" << std::endl;
//					std::string txData = oss.str();
//					arduinoSerial.write(txData.c_str());
//				}
//				else if (currentTBMTrimState == TBM_STATE_TRIM_ON) {
//					std::ostringstream oss;
//					oss << "M" << std::endl;
//					std::string txData = oss.str();
//					arduinoSerial.write(txData.c_str());
//				}
//				else if (currentTBMTrimState == TBM_STATE_AUTOPILOT_ON) {
//					std::ostringstream oss;
//					oss << "A" << std::endl;
//					std::string txData = oss.str();
//					arduinoSerial.write(txData.c_str());
//				}
//				else if (currentTBMTrimState == TBM_STATE_TRIM_ON_MANUAL_PITCH_ON) {
//					std::ostringstream oss;
//					oss << "O" << std::endl;
//					std::string txData = oss.str();
//					arduinoSerial.write(txData.c_str());
//				}
//				prevState = currentTBMTrimState;
//			}
//
//			if (currAutoPilotMode != prevAutoPilotMode && currAutoPilotMode == MANUAL_PILOT || currAutoPilotMode == MANUAL_PILOT_2) {
//
//				//NORMAL NON-TBM STUFF
//				std::ostringstream oss;
//				oss << "M" << std::endl;
//				std::string txData = oss.str();
//				arduinoSerial.write(txData.c_str());
//
//
//			}
//			else if (currAutoPilotMode != prevAutoPilotMode && currAutoPilotMode == AUTO_PILOT) {
//
//				//NORMAL NON-TBM STUFF
//				std::ostringstream oss;
//				oss << "A" << std::endl;
//				std::string txData = oss.str();
//				arduinoSerial.write(txData.c_str());
//				autoPilotDisconnected = false;
//
//			}
//			else if (currAutoPilotMode != prevAutoPilotMode && currAutoPilotMode == YOKE_TRIM_MODE) {
//
//				std::ostringstream oss;
//				oss << "O" << std::endl;
//				std::string txData = oss.str();
//				arduinoSerial.write(txData.c_str());
//
//			}
//			else {
//				std::cerr << "Failed to send data to COM port" << std::endl;
//			}
//			prevAutoPilotMode = currAutoPilotMode;
//
//
//			receivedData = arduinoSerial.read(arduinoSerial.available());
//
//
//			//
//			if (currTrimDown != prevTrimDown) {
//				std::ostringstream oss;
//				float trimWheelValf = XPLMGetDataf(gDataRef);
//				oss << trimWheelValf;
//				std::string trimWheelValInStr = oss.str();
//				trimWheelValInStr.append("\n");
//				XPLMDebugString(trimWheelValInStr.c_str());
//				arduinoSerial.write(trimWheelValInStr.c_str());
//				prevTrimDown = currTrimDown;
//			}
//		}
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
//}

// ******************** END OF FUEL SELECTOR FUNCTIONS *****************//

PLUGIN_API int XPluginStart(
	char* outName,
	char* outSig,
	char* outDesc)
{

	int			mySubMenuItem;
	userAircraft = "";
	planeLoadedFully = false;
	/* First we put a new menu item into the plugin menu.
	 * This menu item will contain a submenu for us. */
	mySubMenuItem = XPLMAppendMenuItem(
		XPLMFindPluginsMenu(),	/* Put in plugins menu */
		"RoninDevGru",				/* Item Title */
		0,						/* Item Ref */
		1);						/* Force English */

	/* Now create a submenu attached to our menu item. */
	myMenu = XPLMCreateMenu(
		"Trim Wheel V1",
		XPLMFindPluginsMenu(),
		mySubMenuItem, 			/* Menu Item to attach to. */
		MyMenuHandlerCallback,	/* The handler */
		0);						/* Handler Ref */
	twMenuIdx = XPLMAppendMenuItem(
		myMenu,
		"TrimWheel [OFFLINE]",
		(void*)0,
		1);
	fsMenuIdx = XPLMAppendMenuItem(
		myMenu,
		"Fuel Selector [OFFLINE]",
		(void*)0,
		1);

	aboutMenuItemIndex = XPLMAppendMenuItem(myMenu, "About Us", (void*)"about", 1);

	std::string comPort = enumerate_ports(TW_PLUGIN);  // Change this to the detected COM port number
	prevAutoPilotMode = 0;
	//	// Initialize the serial port
	if (comPort.empty()) {
		std::ostringstream oss;
		oss << "[RoninDevGru] TrimWheel Module not found." << std::endl;
		XPLMDebugString(oss.str().c_str());
		XPLMSetMenuItemName(myMenu, twMenuIdx, "Trim Wheel [OFFLINE]", 1);
		//return 0; // Failed to find Arduino Micro, till try to start plugin
	}
	else {
		if (!initSerialPort(comPort, 250000)) {
			std::ostringstream oss;
			oss << "[RoninDevGru] Cannot communicate to TrimWheel Module. Please check the connections." << std::endl;
			XPLMDebugString(oss.str().c_str());
			XPLMSetMenuItemName(myMenu, twMenuIdx, "Trim Wheel [OFFLINE]", 1);
		}
		else {
			std::ostringstream oss;
			oss << "[RoninDevGru] TrimWheel Module connected. Waiting for XP to load flight." << std::endl;
			XPLMDebugString(oss.str().c_str());
			XPLMSetMenuItemName(myMenu, twMenuIdx, "Trim Wheel [ONLINE]", 1);
			arduinoConnected = true;
		}
	}

	

	if (arduinoSerial.isOpen()) {


		prevState = -1;	// bug fix for non-initialized AP Trim Switch
		std::ostringstream oss;
		oss << "M" << std::endl;
		std::string txData = oss.str();
		arduinoSerial.write(txData.c_str()); // Send Manual Trigger // Release Stepper

		// Start the serial communication thread for TRIM WHEEL
		//trimWheelActive = true;

		autoPilotDisconnected = false;
		XPLMSetMenuItemName(myMenu,twMenuIdx, "Trim Wheel [ONLINE]", 1);
	}

	/* Provide our plugin's profile to the plugin system. */
	strcpy(outName, "RoninDevGru");
	strcpy(outSig, "com.ronindevgru.twv1");
	strcpy(outDesc, "A plugin for trim wheel module.");



	// ** FUEL SELECTOR INITIALIZATION ****

	std::string comPort2 = enumerate_ports(FS_PLUGIN);

	if (comPort2.empty()) {
		std::ostringstream oss;
		oss << "[RoninDevGru] FuelSelector Module not found." << std::endl;
		XPLMDebugString(oss.str().c_str());
		XPLMSetMenuItemName(myMenu, fsMenuIdx, "Fuel Selector [OFFLINE]", 1);
	}
	else {

		if (!initSerialPort(comPort2, 250000, FS_PLUGIN)) {
			std::ostringstream oss;
			oss << "[RoninDevGru] FuelSelector Module is not responding. Please check your connections." << std::endl;
			XPLMDebugString(oss.str().c_str());
			XPLMSetMenuItemName(myMenu, fsMenuIdx, "Fuel Selector [OFFLINE]", 1);
		}
		else {
			// TODO: MUST CHECK FOR TBM900 ONLY
			//tbm900/switches/fuel/auto_man 1 = man, 0 = auto
			//tbm900 / switches / fuel / fuel_sel
			std::ostringstream oss;
			oss << "[RoninDevGru] FuelSelector Module connected. Waiting for XP to load flight." << std::endl;
			XPLMDebugString(oss.str().c_str());

			//fuelSelectorInitCompleted = false;
			//std::thread(serialPollingThreadFuelSelector).detach();

			fuelLeftCommand = XPLMFindCommand("sim/fuel/fuel_selector_lft");
			if (fuelLeftCommand != nullptr) {
				XPLMRegisterCommandHandler(fuelLeftCommand, fuelLeftCommandHandler, 1, nullptr);
			}

			fuelRightCommand = XPLMFindCommand("sim/fuel/fuel_selector_rgt");
			if (fuelRightCommand != nullptr) {
				XPLMRegisterCommandHandler(fuelRightCommand, fuelRightCommandHandler, 1, nullptr);
			}

			fuelOffCommand = XPLMFindCommand("sim/fuel/fuel_selector_none");
			if (fuelOffCommand != nullptr) {
				XPLMRegisterCommandHandler(fuelOffCommand, fuelOffCommandHandler, 1, nullptr);
			}
			XPLMSetMenuItemName(myMenu, fsMenuIdx, "Fuel Selector [ONLINE]", 1);
			prevFuelSelectorPosition = 1.0;
			//fuelSelectorActive = true;
		}

	}

	//std::thread(serialCommsThreadTW).detach();
	XPLMRegisterFlightLoopCallback(MainFlightLoop, -1.0f, nullptr);
	/* Only return that we initialized correctly if we found the data ref. */
	//return (gDataRef != NULL) ? 1 : 0;
	return 1;
}


PLUGIN_API void	XPluginStop(void)
{
// ****** TRIM WHEEL RELATED GARBAGE COLLECTION ******** ////
	if (arduinoSerial.isOpen()) {
		std::ostringstream oss;
		oss << "M" << std::endl;
		std::string txData = oss.str();
		arduinoSerial.write(txData.c_str()); // Send Manual Trigger
	}
	keepRunning.store(false);  // Signal the thread to stop

	// Unregister the flight loop callback

	XPLMUnregisterFlightLoopCallback(FlightLoopCallback, nullptr);
	XPLMUnregisterFlightLoopCallback(MainFlightLoop, nullptr);
	// Stop the serial communication thread
	if (arduino.isOpen()) {
		trimWheelActive = false;
		arduino.close();
	}


	XPLMCommandRef pitchTrimUpCommand = XPLMFindCommand("sim/flight_controls/pitch_trim_up");
	if (pitchTrimUpCommand != nullptr) {
		// Unregister the command handler
		XPLMUnregisterCommandHandler(pitchTrimUpCommand, PitchTrimUpCommandHandler, 1, nullptr);
	}
	XPLMCommandRef pitchTrimDownCommand = XPLMFindCommand("sim/flight_controls/pitch_trim_down");
	if (pitchTrimDownCommand != nullptr) {
		// Unregister the command handler
		XPLMUnregisterCommandHandler(pitchTrimDownCommand, PitchTrimDownCommandHandler, 1, nullptr);
	}
	// Close serial port
	if (arduinoSerial.isOpen()) {
		arduinoSerial.close();
	}
// END OF TRIMWHEEL GARBAGE COLLECTION

// ****** FUEL SELECTOR GARBAGE COLLECTION


	keepRunningFuelSelector.store(false);  // Signal the thread to stop

	// Stop the serial communication thread
	if (fuelSelectorSerial.isOpen()) {
		fuelSelectorActive = false;
		std::ostringstream oss;
		oss << "T";
		std::string fuelModeStr = oss.str();
		fuelModeStr.append("\n");
		//XPLMDebugString(fuelModeStr.c_str());
		fuelSelectorSerial.write(fuelModeStr.c_str()); // Send DISENGAGE command
		fuelSelectorSerial.close();
	}


// END OF FUEL SELECTOR GARBAGE COLLECTION
}

PLUGIN_API void XPluginDisable(void)
{
// TRIM WHEEL DISABLE FUNCTIONS
	if (arduinoSerial.isOpen()) {
		std::ostringstream oss;
		oss << "M" << std::endl;
		std::string txData = oss.str();
		arduinoSerial.write(txData.c_str()); // Send Manual Trigger

	}
	keepRunning.store(false);  // Signal the thread to stop
	XPLMUnregisterFlightLoopCallback(FlightLoopCallback, NULL);
	XPLMUnregisterFlightLoopCallback(MainFlightLoop, nullptr);
	//// Stop the serial communication thread

	//// Close serial port
	if (arduinoSerial.isOpen()) {
		arduinoSerial.close();
		XPLMSetMenuItemName(myMenu, twMenuIdx, "Trim Wheel [OFFLINE]", 1);
		trimWheelActive = false;
	}
//  END OF TRIMWHEEL DISABLE FUNCTIONS
// FUEL SELECTOR DISABLE FUNCTION
	keepRunningFuelSelector.store(false);  // Signal the thread to stop

	//// Stop the serial communication thread

	//// Close serial port
	if (fuelSelectorSerial.isOpen()) {
		XPLMSetMenuItemName(myMenu, fsMenuIdx, "Fuel Selector [OFFLINE]", 1);
		fuelSelectorSerial.close();
		fuelSelectorActive = false;
	}
// END OF FUEL SELECTOR DISABLE FUNCTION
}

PLUGIN_API int XPluginEnable(void)
{
	
// TRIM WHEEL ENABLE FUNCTIONS
	std::ostringstream oss;
	oss << "[RoninDevGru] PLUGIN ENABLED." << std::endl;
	XPLMDebugString(oss.str().c_str());
	if (!arduinoSerial.isOpen()) {
		std::string comPort = enumerate_ports();  // Change this to the detected COM port number

		if (comPort.empty()) {
			std::ostringstream oss;
			oss << "[RoninDevGru] TrimWheel Module not found." << std::endl;
			XPLMDebugString(oss.str().c_str());
			XPLMSetMenuItemName(myMenu, twMenuIdx, "Trim Wheel [OFFLINE]", 1);
			//return 0; // Failed to find Arduino Micro, till try to start plugin
		}
		else {
			if (!initSerialPort(comPort, 250000)) {
				std::ostringstream oss;
				oss << "[RoninDevGru] Cannot communicate to TrimWheel Module. Please check the connections." << std::endl;
				XPLMDebugString(oss.str().c_str());
				XPLMSetMenuItemName(myMenu, twMenuIdx, "Trim Wheel [OFFLINE]", 1);
			}
			else {
				std::ostringstream oss;
				oss << "[RoninDevGru] TrimWheel Module connected. Waiting for XP to load flight." << std::endl;
				XPLMDebugString(oss.str().c_str());
				XPLMSetMenuItemName(myMenu, twMenuIdx, "Trim Wheel [ONLINE]", 1);
				arduinoConnected = true;
				prevState = -1;	// bug fix for non-initialized AP Trim Switch
				prevAutoPilotMode = 0;
				trimWheelActive = true;

			}
		}

	}
// END OF TRIMWHEEL ENABLE FUNCTIONS

// FUEL SELECTOR ENABLE FUNCTIONS
	if (!fuelSelectorSerial.isOpen()) {
		std::string comPort = enumerate_ports(FS_PLUGIN);  // Change this to the detected COM port number


		if (comPort.empty()) {
			std::ostringstream oss;
			oss << "[RoninDevGru] FuelSelector Module not found." << std::endl;
			XPLMDebugString(oss.str().c_str());
			XPLMSetMenuItemName(myMenu, fsMenuIdx, "Fuel Selector [OFFLINE]", 1);
		}
		else {
			if (!initSerialPort(comPort, 250000, FS_PLUGIN)) {
				std::ostringstream oss;
				oss << "[RoninDevGru] FuelSelector Module is not responding. Please check your connections." << std::endl;
				XPLMDebugString(oss.str().c_str());
				XPLMSetMenuItemName(myMenu, fsMenuIdx, "Fuel Selector [OFFLINE]", 1);
			}
			else {
				XPLMSetMenuItemName(myMenu, fsMenuIdx, "Fuel Selector [ONLINE]", 1);
				keepRunningFuelSelector.store(true);
				fuelSelectorInitCompleted = false;
				gFuelSelectorDataRef = XPLMFindDataRef("tbm900/switches/fuel/fuel_sel");
				//std::thread(serialPollingThreadFuelSelector).detach();
				if (gFuelSelectorDataRef != NULL) {
					
					currFuelSelectorPosition = 0.0;
					prevFuelSelectorPosition = 0.0;
					float fuelSelectorPositionF = XPLMGetDataf(gFuelSelectorDataRef);
					std::ostringstream oss;
					oss << fuelSelectorPositionF;
					std::string fuelSelectorPositionInVal = oss.str();
					fuelSelectorPositionInVal.append("\n");
					//XPLMDebugString(fuelSelectorPositionInVal.c_str());
					fuelSelectorSerial.write(fuelSelectorPositionInVal.c_str()); // Send Manual Trigger
					fuelSelectorActive = true;
				}
			}

		}

	}
	//keepRunning.store(true);
	//keepRunningFuelSelector.store(true);
	//std::thread(serialCommsThreadFS).detach();
	//std::thread(serialCommsThreadTW).detach();
	//XPLMRegisterFlightLoopCallback(FlightLoopCallback, -1.0f, nullptr);
	//XPLMRegisterFlightLoopCallback(MainFlightLoop, -1.0f, nullptr);
// END OF FUEL SELECTOR ENABLE FUNCTIONS
	return 1;
}

PLUGIN_API void XPluginReceiveMessage(
	XPLMPluginID	inFromWho,
	int				inMessage,
	void* inParam)
{
	if (inMessage == XPLM_MSG_PLANE_LOADED) {
		// This message indicates the aircraft is fully loaded
		planeLoadedFully = true;
		XPLMDebugString("Aircraft fully loaded.\n");
	}
}
// Function to create the "About Us" window
void createAboutWindow() {
	// Define the window's bounds
	int left = 100;
	int top = 600;
	int right = 400;
	int bottom = 400;

	// Create the window
	aboutWindow = XPLMCreateWindow(
		left, top, right, bottom,       // Window bounds
		1,                              // Start visible
		drawAboutWindow,                // Draw callback
		NULL,               // Mouse click callback for close button
		handleMouseClick,                           // Key callback (optional)
		NULL                          // Cursor status callback (optional)
		
	);
}

// Function to draw the "About Us" window with a close button
void drawAboutWindow(XPLMWindowID inWindowID, void* inRefcon) {
	// Get the window's current bounds
	int left, top, right, bottom;
	XPLMGetWindowGeometry(inWindowID, &left, &top, &right, &bottom);

	// Draw a translucent dark background box
	XPLMDrawTranslucentDarkBox(left, top, right, bottom);

	// Define the content of the "About Us" window
	const char* aboutText = "Plugin by RoninDevGru";
	float white[] = {1.0, 1.0, 1.0};
	// Draw the text inside the window
	XPLMDrawString(white , left + 20, top - 30, (char*)aboutText, NULL, xplmFont_Proportional);

	// Draw the "Close" button as a small rectangle with text
	const int buttonWidth = 50;
	const int buttonHeight = 20;
	int buttonLeft = right - buttonWidth - 10;
	int buttonBottom = bottom + 10;

	XPLMDrawTranslucentDarkBox(buttonLeft, buttonBottom + buttonHeight, buttonLeft + buttonWidth, buttonBottom);

	// Draw "Close" text on the button
	
	XPLMDrawString(white, buttonLeft + 5, buttonBottom + 5, (char*)"Close", NULL, xplmFont_Proportional);
}

// Function to handle mouse clicks for closing the window
int handleMouseClick(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void* inRefcon) {
	if (inMouse == xplm_MouseDown) {
		// Get the window's current bounds
		int left, top, right, bottom;
		XPLMGetWindowGeometry(inWindowID, &left, &top, &right, &bottom);

		// Define close button bounds
		const int buttonWidth = 50;
		const int buttonHeight = 20;
		int buttonLeft = right - buttonWidth - 10;
		int buttonBottom = bottom + 10;

		// Check if click is inside the close button area
		if (x >= buttonLeft && x <= buttonLeft + buttonWidth &&
			y >= buttonBottom && y <= buttonBottom + buttonHeight) {
			closeAboutWindow();
			return 1;  // Indicate that the click was handled
		}
	}
	return 0;  // Click not handled
}


// Function to close the window
void closeAboutWindow() {
	if (aboutWindow != NULL) {
		XPLMDestroyWindow(aboutWindow);
		aboutWindow = NULL;
	}
}



void	MyMenuHandlerCallback(
	void* inMenuRef,
	void* inItemRef)
{
	/* This is our handler for the menu item.  Our inItemRef is the refcon
	 * we registered in our XPLMAppendMenuItem calls.  It is either +1000 or
	 * -1000 depending on which menu item is picked. */
	if (inItemRef == (void*)"about") {
		// Create and display the About Us window if not already open
		if (aboutWindow == NULL) {
			createAboutWindow();
		}
	}
}
