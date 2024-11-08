
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include "XPLMMenus.h"
#include "XPLMPlanes.h"
#include <thread>
#include <atomic>
#include "serial.h"
#include <mutex>
#include <vector>
#include <cmath>
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

//************************************
// FUEL SELECTOR DEFINITIONS

serial::Serial fuelSelectorSerial;
std::atomic<bool> keepRunningFuelSelector(true);
static XPLMDataRef		gFuelSelectorDataRef = NULL;
static XPLMDataRef		gFuelModeDataRef = NULL;
float currFuelSelectorPosition = 0.0;
float prevFuelSelectorPosition = 0.0;
int fuelSelectorState = 0;
int prevFuelSelectorState = 0;
XPLMCommandRef fuelLeftCommand;
XPLMCommandRef fuelRightCommand;
XPLMCommandRef fuelOffCommand;

bool fuelLeftActive = false;
bool fuelRightActive = false;
bool fuelOffActive = false;
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

static std::string trimWheelValInStr;
float prevTrimDown = 0.0f;
float currTrimDown = 0.0f;
int prevTrimUp = 0;
int currTrimUp = 0;
int prevAutoPilotMode = 0;
int prevAPModeTBM = 0;
int prevState = 0;
int currentTBMTrimState = 0;
int autoPilotModeNormal = 0;

bool hasSentZeroDown = false;
bool hasSentZeroUp = false;

bool pitchTrimUpActive = false;
bool pitchTrimDownActive = false;
bool autoPilotDisconnected = false;
bool triggerAP = false;

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
			if (device.hardware_id.find("VID_4262&PID_3501") != std::string::npos) {

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
	triggerAPDisc.store(true);
}

float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon) {
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

// Thread function for polling and sending data via serial
void serialPollingThread() {
	while (keepRunning.load()) {
		if (!hasLoadedAircraftName && isInFlight()) {
			//XPLMDebugString("MODEL IS\n");
			userAircraft = getFirstAircraftModel();
			hasLoadedAircraftName = true;

			pitchTrimUpCommand = XPLMFindCommand("sim/flight_controls/pitch_trim_up");
			pitchTrimDownCommand = XPLMFindCommand("sim/flight_controls/pitch_trim_down");
			autopilotDisconnectCommand = XPLMFindCommand("sim/autopilot/disconnect");

		}
		//if (!arduinoConnected) continue;
		// tbm900/actuators/ap/disc 
		// sim/cockpit/autopilot/autopilot_mode = 2 else 1
		//tbm900/switches/elec/ap_trims OFF = 0, AP OFF = 1, ON = 2
		// sim/flight_controls/pitch_trim_down
		// sim/flight_controls/pitch_trim_up
		try {
			if (hasLoadedAircraftName && userAircraft == "TBM900.acf") {
				gDataRefTBM = XPLMFindDataRef("tbm900/switches/elec/ap_trims_raw");
				gDataRef = XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode");

				XPLMCommandRef pitchTrimUpCommand = XPLMFindCommand("sim/flight_controls/pitch_trim_up");
				XPLMCommandRef pitchTrimDownCommand = XPLMFindCommand("sim/flight_controls/pitch_trim_down");


				int tbmTrimMode = XPLMGetDatai(gDataRefTBM);
				int apModeTBM = XPLMGetDatai(gDataRef);
				int currentState = -1;

				if (tbmTrimMode == TBM_STATE_TRIM_OFF) {
					currentState = TBM_STATE_TRIM_OFF;
					currentTBMTrimState = currentState; // save current state;
					//autoPilotDisconnected = false;
				}
				else if (tbmTrimMode == TBM_STATE_TRIM_ON) {
					currentState = TBM_STATE_TRIM_ON;
					currentTBMTrimState = currentState; // save current state;
					//autoPilotDisconnected = false;
				}

				if (apModeTBM == TBM_AP_ON) {
					currentState = TBM_STATE_AUTOPILOT_ON;
					currentTBMTrimState = currentState; // save current state;
					autoPilotDisconnected = false;
				}

				if (pitchTrimDownActive || pitchTrimUpActive) {

					currentState = TBM_STATE_TRIM_ON_MANUAL_PITCH_ON;
					//autoPilotDisconnected = false;

				}
				else {
					currentState = currentTBMTrimState;
				}

				if (arduinoSerial.isOpen()) {
					if (currentState != prevState) {
						std::ostringstream oss;
						oss << "[RoninDevGru] STATE:" << currentState << std::endl;
						XPLMDebugString(oss.str().c_str());
						if (currentState == TBM_STATE_TRIM_OFF) {
							std::ostringstream oss;
							oss << "N" << std::endl;
							std::string txData = oss.str();
							arduinoSerial.write(txData.c_str());
						}
						else if (currentState == TBM_STATE_TRIM_ON) {
							std::ostringstream oss;
							oss << "M" << std::endl;
							std::string txData = oss.str();
							arduinoSerial.write(txData.c_str());
						}
						else if (currentState == TBM_STATE_AUTOPILOT_ON) {
							std::ostringstream oss;
							oss << "A" << std::endl;
							std::string txData = oss.str();
							arduinoSerial.write(txData.c_str());
						}
						else if (currentState == TBM_STATE_TRIM_ON_MANUAL_PITCH_ON) {
							std::ostringstream oss;
							oss << "O" << std::endl;
							std::string txData = oss.str();
							arduinoSerial.write(txData.c_str());
						}
						prevState = currentState;
					}
				}

			}
			else {
				gDataRef = XPLMFindDataRef("sim/cockpit/autopilot/autopilot_mode");
				if (gDataRef != NULL)
				{
					int autoPilotMode = XPLMGetDatai(gDataRef);

					if (pitchTrimDownActive || pitchTrimUpActive) {
						autoPilotMode = YOKE_TRIM_MODE;
					}

					if (arduinoSerial.isOpen() && autoPilotMode != prevAutoPilotMode && autoPilotMode == MANUAL_PILOT || autoPilotMode == MANUAL_PILOT_2) {

						//NORMAL NON-TBM STUFF
						std::ostringstream oss;
						oss << "M" << std::endl;
						std::string txData = oss.str();
						arduinoSerial.write(txData.c_str());


					}
					else if (arduinoSerial.isOpen() && autoPilotMode != prevAutoPilotMode && autoPilotMode == AUTO_PILOT) {

						//NORMAL NON-TBM STUFF
						std::ostringstream oss;
						oss << "A" << std::endl;
						std::string txData = oss.str();
						arduinoSerial.write(txData.c_str());
						autoPilotDisconnected = false;

					}
					else if (arduinoSerial.isOpen() && autoPilotMode != prevAutoPilotMode && autoPilotMode == YOKE_TRIM_MODE) {

						std::ostringstream oss;
						oss << "O" << std::endl;
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
			}

			if (arduinoSerial.isOpen()) {
				if (arduinoSerial.available()) {
					std::string receivedData = arduinoSerial.read(arduinoSerial.available());
					int receivedEncoderDirF = std::stof(receivedData);
					std::ostringstream oss;
					oss << "Received: " << receivedData << std::endl;
					std::string rxData = oss.str();
					//XPLMDebugString(rxData.c_str());
					//tbm900/switches/elec/ap_trims OFF = 0, AP OFF = 1, ON = 2
					gDataRef = XPLMFindDataRef("sim/flightmodel/controls/elv_trim");
					if (gDataRef != NULL)
					{
						std::ostringstream oss3;
						float trimWheelValf = XPLMGetDataf(gDataRef);
						if (receivedEncoderDirF == 1) {
							trimWheelValf += 0.0025;
							if (trimWheelValf > 1.0) {
								trimWheelValf = 1.0;
							}
							XPLMSetDataf(gDataRef, trimWheelValf);
						}
						else if (receivedEncoderDirF == -1) {

							trimWheelValf -= 0.0025;
							if (trimWheelValf < -1.0) {
								trimWheelValf = -1.0;
							}
							XPLMSetDataf(gDataRef, trimWheelValf);
						}
						else if (receivedEncoderDirF == 2) {
							triggerAPDisc.store(true);

						}




					}
				}
			}
			else {
				XPLMDebugString("[RoninDevGru] Serial communication is CLOSED");
			}


			std::ostringstream oss;
			std::ostringstream oss2;

			gDataRef = XPLMFindDataRef("sim/flightmodel/controls/elv_trim");
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
						XPLMDebugString(trimWheelValInStr.c_str());
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
			oss << "[RoninDevGru] Error opening serial port: " << e.what() << std::endl;
			XPLMDebugString(oss.str().c_str());
		}

	}

	std::ostringstream oss;
	oss << "[RoninDevGru] Serial communication thread exiting... " << std::endl;

	XPLMDebugString(oss.str().c_str());
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

// Command handler function
int PitchTrimUpCommandHandler(XPLMCommandRef command, XPLMCommandPhase phase, void* refcon) {
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

// Thread function for polling and sending data via serial
void serialPollingThreadFuelSelector() {
	while (keepRunningFuelSelector.load()) {
		if (!hasLoadedAircraftName && isInFlight()) {
			userAircraft = getFirstAircraftModel();
			hasLoadedAircraftName = true;
			gFuelSelectorDataRef = XPLMFindDataRef("tbm900/switches/fuel/fuel_sel");
			gFuelModeDataRef = XPLMFindDataRef("tbm900/switches/fuel/auto_man");
			if (fuelSelectorSerial.isOpen()) {
				if (gFuelSelectorDataRef != NULL) {
					std::ostringstream oss;

					int fuelMode = XPLMGetDatai(gFuelModeDataRef);
					if (fuelMode == 0) {
						oss << "R";
						std::string fuelModeStr = oss.str();
						fuelModeStr.append("\n");
						XPLMDebugString(fuelModeStr.c_str());
						fuelSelectorSerial.write(fuelModeStr.c_str()); // Send Manual Trigger
					}
					else if (fuelMode == 1) {
						oss << "T";
						std::string fuelModeStr = oss.str();
						fuelModeStr.append("\n");
						XPLMDebugString(fuelModeStr.c_str());
						fuelSelectorSerial.write(fuelModeStr.c_str()); // Send Manual Trigger
					}


				}
			}
		}

		try {
			if (hasLoadedAircraftName && userAircraft == "TBM900.acf") {
				std::ostringstream oss;
				std::ostringstream oss2;

				if (gFuelSelectorDataRef != NULL)
				{
					currFuelSelectorPosition = XPLMGetDataf(gFuelSelectorDataRef);
					//if (fmod(currFuelSelectorPosition, 0.1) == 0) {
						if (currFuelSelectorPosition != prevFuelSelectorPosition) {

							if (fuelSelectorSerial.isOpen()) {
								if (gFuelSelectorDataRef != NULL) {
				
									float fuelSelectorPositionF = XPLMGetDataf(gFuelSelectorDataRef);
									std::ostringstream oss;
									oss << fuelSelectorPositionF;
									std::string fuelSelectorPositionInVal = oss.str();
									fuelSelectorPositionInVal.append("\n");
									//XPLMDebugString(fuelSelectorPositionInVal.c_str());
									fuelSelectorSerial.write(fuelSelectorPositionInVal.c_str()); // Send Manual Trigger

								}
							}
							prevFuelSelectorPosition = currFuelSelectorPosition;
						}

					//}

				}

				gFuelModeDataRef = XPLMFindDataRef("tbm900/switches/fuel/auto_man");
				if (fuelSelectorSerial.isOpen()) {
					if (gFuelSelectorDataRef != NULL) {
						std::ostringstream oss;
						int fuelMode = XPLMGetDatai(gFuelModeDataRef);
						if (fuelMode != prevFuelSelectorState) {
							if (fuelMode == 0) {
								oss << "R";
								std::string fuelModeStr = oss.str();
								fuelModeStr.append("\n");
								//XPLMDebugString(fuelSelectorPositionInVal.c_str());
								fuelSelectorSerial.write(fuelModeStr.c_str()); // Send Manual Trigger
								prevFuelSelectorPosition = 0.0; // reset to OFF for homing
							}
							else if (fuelMode == 1) {
								oss << "T";
								std::string fuelModeStr = oss.str();
								fuelModeStr.append("\n");
								//XPLMDebugString(fuelSelectorPositionInVal.c_str());
								fuelSelectorSerial.write(fuelModeStr.c_str()); // Send Manual Trigger
							}
						}

						prevFuelSelectorState = fuelMode;

					}
				}
			}

			// Sleep or wait a bit to avoid overwhelming the serial communication
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		catch (serial::IOException& e) {
			std::ostringstream oss;
			oss << "[RoninDevGru] Error opening serial port: " << e.what() << std::endl;
			XPLMDebugString(oss.str().c_str());
		}
	}
}

// ******************** END OF FUEL SELECTOR FUNCTIONS *****************//

PLUGIN_API int XPluginStart(
	char* outName,
	char* outSig,
	char* outDesc)
{



	XPLMMenuID	myMenu;
	int			mySubMenuItem;
	std::string comPort = enumerate_ports(TW_PLUGIN);  // Change this to the detected COM port number
	prevAutoPilotMode = 0;
	//	// Initialize the serial port
	if (comPort.empty()) {
		//return 0; // Failed to find Arduino Micro, till try to start plugin
	}
	else {
		if (!initSerialPort(comPort, 250000)) {
			//return 0; // Failed to initialize serial, still try to start plugin for the other module
		}
		else {
			arduinoConnected = true;
		}
	}

	

	if (arduinoSerial.isOpen()) {
		std::ostringstream oss;
		oss << "M" << std::endl;
		std::string txData = oss.str();
		arduinoSerial.write(txData.c_str()); // Send Manual Trigger // Release Stepper

		// Start the serial communication thread for TRIM WHEEL
		std::thread(serialPollingThread).detach();  // Detach the thread so it runs independently

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

		autoPilotDisconnected = false;
	}

	/* Provide our plugin's profile to the plugin system. */
	strcpy(outName, "RoninDevGru");
	strcpy(outSig, "com.ronindevgru.twv1");
	strcpy(outDesc, "A plugin for trim wheel module.");

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
	/* Append a few menu items to our submenu.  We will use the refcon to
 * store the amount we want to change the radio by. */
 //XPLMAppendMenuItem(
 //	myMenu,
 //	"SEND Message",
 //	(void*)-1000,
 //	1);
 //XPLMAppendMenuItem(
 //	myMenu,
 //	"AP Disc",
 //	(void*)+1000,
 //	1);
	//tbm900/switches/fuel/fuel_sel 0.0 = off, r = 1.0, l = 2
	//sim/fuel/fuel_selector_lft
	//sim/fuel/fuel_selector_rgt
	//sim/fuel/fuel_selector_none


	// ** FUEL SELECTOR INITIALIZATION ****

	std::string comPort2 = enumerate_ports(FS_PLUGIN);

	if (comPort2.empty()) {
		//return 0; // Failed to find Arduino Micro,try to start plugin
	}

	if (!initSerialPort(comPort2, 250000, FS_PLUGIN)) {
		return 0; // Failed to initialize serial, do not start plugin
	}
	else {
		// TODO: MUST CHECK FOR TBM900 ONLY
		//tbm900/switches/fuel/auto_man 1 = man, 0 = auto
		//tbm900 / switches / fuel / fuel_sel
		std::thread(serialPollingThreadFuelSelector).detach();

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

	}




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
	// Stop the serial communication thread
	if (arduino.isOpen()) {
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
	//// Stop the serial communication thread

	//// Close serial port
	if (arduinoSerial.isOpen()) {
		arduinoSerial.close();
	}
//  END OF TRIMWHEEL DISABLE FUNCTIONS
// FUEL SELECTOR DISABLE FUNCTION
	keepRunningFuelSelector.store(false);  // Signal the thread to stop

	//// Stop the serial communication thread

	//// Close serial port
	if (fuelSelectorSerial.isOpen()) {
		fuelSelectorSerial.close();
	}
// END OF FUEL SELECTOR DISABLE FUNCTION
}

PLUGIN_API int XPluginEnable(void)
{
// TRIM WHEEL ENABLE FUNCTIONS
	if (!arduinoSerial.isOpen()) {
		std::string comPort = enumerate_ports();  // Change this to the detected COM port number

		// Initialize the serial port
		if (comPort.empty()) {
			
		}
		else {
			if (!initSerialPort(comPort, 250000)) {
			}
			prevAutoPilotMode = 0;
			keepRunning.store(true);
			std::thread(serialPollingThread).detach();

		}

	}
// END OF TRIMWHEEL ENABLE FUNCTIONS

// FUEL SELECTOR ENABLE FUNCTIONS
	if (!fuelSelectorSerial.isOpen()) {
		std::string comPort = enumerate_ports(FS_PLUGIN);  // Change this to the detected COM port number

		// Initialize the serial port
		if (comPort.empty()) {
			return 0; // Failed to find Arduino Micro, do not start plugin
		}

		if (!initSerialPort(comPort, 250000, FS_PLUGIN)) {
			return 0; // Failed to initialize serial, do not start plugin
		}
		keepRunningFuelSelector.store(true);
		std::thread(serialPollingThreadFuelSelector).detach();
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

		}
	}
// END OF FUEL SELECTOR ENABLE FUNCTIONS
	return 1;
}

PLUGIN_API void XPluginReceiveMessage(
	XPLMPluginID	inFromWho,
	int				inMessage,
	void* inParam)
{

}


void	MyMenuHandlerCallback(
	void* inMenuRef,
	void* inItemRef)
{
	/* This is our handler for the menu item.  Our inItemRef is the refcon
	 * we registered in our XPLMAppendMenuItem calls.  It is either +1000 or
	 * -1000 depending on which menu item is picked. */
	ExecutePitchTrimUp();
}
