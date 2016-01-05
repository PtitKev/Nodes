/// Ydle.h
///
/// Ydle implementation for Arduino
/// See the README file in this directory for documentation
///
///
/// Authors:
/// Fabrice Scheider AKA Denia,
/// Manuel Esteban AKA Yaug
/// Matthieu Desgardin AKA Zescientist
/// Yargol AKA Yargol
/// Xylerk
/// PtitKev
///
/// WebPage: http://www.ydle.fr/index.php
/// Contact: http://forum.ydle.fr/index.php
/// Licence: CC by sa (http://creativecommons.org/licenses/by-sa/3.0/fr/)
/// Pll function inspired on VirtualWire library
///
/// \Mainpage Ydle library for Arduino
///
/// \Installation
/// To install, unzip the library into the libraries sub-directory of your
/// Arduino application directory. Then launch the Arduino environment; you
/// should see the library in the Sketch->Import Library menu, and example
/// code in File->Sketchbook->Examples->Ydle menu.
///
/// \Revision History:
///
/// \version 0.5.2 2014-02-18
///		- Autoconvert value for DATA_DEGREEC, DATA_DEGREEF and DATA_HUMIDITY
///
/// \version 0.5.1 2014-01-17
/// 	- Partial rewriting of code
/// 	- Using timer interrupt for call the pll function
///     - Add callback function to handle user command
///     - Rename all #define to avoid confusion with other lib
///
/// \version 0.5 2013-09-24
/// 	- Now use Pll function to receive signal
/// 	- Partial asynchronous rewrite of the code
///		- Parity bit
/// 	- Variable frame length for more informations
///
/// \version 0.2 2013-08-20
/// 	- Creation of the library
///
/// \version 0.1
///     - Original release of the Node code
///
/// To use the Ydle library, you must have:
///     #include <Ydle.h>
/// At the top of your sketch.

#ifndef YDLE_H
#define YDLE_H

#include <stdlib.h>
#include <Arduino.h>

#if defined(ARDUINO)
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <wiring.h>
#endif
#else // error
#error Platform not defined
#endif

#define _YDLE_DEBUG

#define YDLE_MAX_FRAME 2

#define YDLE_MAX_SIZE_FRAME 64
// 1 sec timeout for ack
#define YDLE_ACK_TIMEOUT 250

#define YDLE_TYPE_STATE				1 // Node send data
#define YDLE_TYPE_CMD						2 // ON/OFF sortie etc...
#define YDLE_TYPE_ACK						3 // Acquit last command
#define YDLE_TYPE_STATE_ACK	4 // Node send data and want ACK

#define YDLE_DATA_BOOL					0 // On - OFF (1 bit / 8 bits data)
#define YDLE_DATA_UINT8				1 // (8 bits / 16 bits data)
#define YDLE_DATA_UINT16			2 // (16 bits / 24 bits data)
#define YDLE_DATA_UINT24			3 // (24 bits / 32 bits data)

#define YDLE_CMD_LINK						0 // Link a node to the master
#define YDLE_CMD_ON							1 // Send a ON command to node data = N° output
#define YDLE_CMD_OFF						2 // Send a OFF command to node data = N° output
#define YDLE_CMD_RESET					3 // Ask a node to reset is configuration
#define YDLE_CMD_SET						4 // Set value
#define YDLE_CMD_GET						5 // Get value

// Défini un type de structure Frame_t
struct Frame_t
{
  uint8_t receptor;	// 8 bytes
  uint8_t sender;		// 8 bytes
  uint8_t type;				// 2 bytes
  uint8_t taille;			// 3 bytes data len + crc in BYTES
  uint8_t data[30];	//
  uint8_t crc;				// 8 bytes
} ;

// Défini un type de structure Config_t
struct Config_t
{
  uint8_t IdMaster;	// Id du master
  uint8_t IdNode;		// Id de la node
} ;

extern "C" {
  // callback function
  typedef void (*ydleCallbackFunction)(Frame_t *frame) ;
}

volatile static Config_t m_Config;
volatile static bool m_bLnkSignalReceived;
volatile static bool m_initializedState; // Indique si le node est initialisé

class ydle
{
  private:
    // On déclare les structures
    Frame_t m_SendFrame;	 // send frame
    Frame_t m_ReceivedFrame;	 // received frame
    Frame_t m_FrameBuffer[YDLE_MAX_FRAME];
    ydleCallbackFunction callback;
    bool _callback_set;

  public:
    static ydle * ydleObj;
    // Le constructeur qui lance une instance avec les numéros des pins de l'émetteur, du récepteur et du boutton
    ydle(int rx, int tx, int button) ;

    // Le constructeur qui lance une instance avec les numéros des pins de l'émetteur, du récepteur et du boutton
    // Par défaut, le récepteur est en 12, l'émetteur en 10 et le boutton en 3
    ydle() ;

    static void timerInterrupt() ;
    void pll() ;

    // Used to read the configuration
    void ReadConfig() ;

    // Envoie des verrous et des bits formant une trame
    uint8_t send(Frame_t *frame) ;

    // New function need to be called by the main function in order to handle the new received frame
    uint8_t receive() ;

    // Ecoute le récepteur pour l'arrivée d'un signal
    void listenSignal() ;

    // Crée une trame avec les infos données en paramètre
    void dataToFrame(Frame_t *frame, unsigned long destination, unsigned long sender, unsigned long type) ;

    // Crée une trame avec le type
    void dataToFrame(Frame_t *frame, unsigned long type) ;

    // extract any type of data from receivedsignal
    int extractData(Frame_t *frame, int index, int &itype, long &ivalue) ;

    // add TYPE_CMD data
    void addCmd(Frame_t *frame, int type, int data) ;

    // Retourne l'état de la Node
    bool initialized() ;

    // Retourne l'état du bouton de reset
    bool resetButton() ;

    //~ int isSignal() ;
    //~ bool isDone() ;

    // CRC calculation
    unsigned char computeCrc(Frame_t *frame) ;

    // Launch the timer for the receive function
    void init_timer() ;

    // Function to attach a user defined function for handle received frame
    void attach(ydleCallbackFunction function) ;

    // Remove EEProm values & change initialized state
    static void resetNode() ;

    // float to half conversion
    union _FP16 floatToHalf(float number) ;

    void addData(Frame_t *frame, int type, bool data) ;
    void addData(Frame_t *frame, int type, int data) ;
    void addData(Frame_t *frame, int type, float data) ;
    //~ void addData(Frame_t *frame, int type, float fdata) ;

  private:
    // Fonctions de débogage
    void log(String msg) ;
    void log(String msg, int i) ;
    // Affiche le contenue d'une trame reçue
    void printFrame(Frame_t *trame) ;

    // Do something with a received Command
    void handleReceivedFrame(Frame_t *frame) ;

    // Compare le signal reçu au signal de référence
    bool checkSignal(Frame_t *frame) ;

    // EEprom handling, to serialize the configuraiton
    void writeEEProm() ;
    void readEEProm() ;

    uint8_t crc8(const uint8_t* buf, uint8_t length) ;
} ;

#endif