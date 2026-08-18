			/***********************************
			 *                                 *
			 *      IDN-Scope for Windows      *
			 *                                 *
			 *    Version 1.07  18-Aug-2026    *
			 *    Start date:    4-Apr-2026    *
			 *                                 *
			 *     Visual Studio 2026 IDE      *
			 *                                 *
			 *   (c) 2026 by Anthony Barrett   *
			 *                                 *
			 ***********************************/

			 
/* 

IDN-Scope is a minimal IDN receiver that displays DAC data in a simulator graphics window.
It supports both UDP and TCP connections.

It can be used to test IDN network connections from existing IDN clients.
It can be used to test and help you develop your own IDN client.
It can be used to test and evaluate my proposed "IDN over TCP" implementation.

To implement an "IDN over TCP" client, just send the same IDN packet data to a TCP connected socket instead.

"IDN over TCP" has the following advantages:

 1. Connection integrity. A TCP connection requires the receiving side to establish and maintain a session.
 2. Data integrity. TCP guarantees data delivery.
 3. Prevents out of order packets.
 4. Prevents lost packets.
 5. No packet size limitations. No need to split large data chunks over several packets because of UDP datagram fragmentation limits.


Most of the functions here are copied from the LASERfIREUSB source to this single minimal source file.

This source file is intended to be built by Visual Studio 2026, as a Windows "Console App" project.

To use this source file, create a new Windows "Console App" project with the name "IDN-Scope", replace the
auto generated source file with this one and then change the "Charater Set" to "Not Set" in the project properties. 

There are some hooks left in this code that can be used to drive real DAC hardware.
You are welcome to add your own code to the functions "ShutterClose()", "ShutterOpen()" and "IDNDACUpdate()" to drive a real DAC

ANSI colour codes are used with some printf() calls to add some colour. These codes may need to be enabled on Windows 10 to display correctly.
This is done using "regedit" - google "how to enable ansi colours in a windows 10 command prompt" to learn how.

*/


#include <stdio.h>
#include <winsock2.h>
#include <WS2tcpip.h>
#include <iphlpapi.h>
#include <windows.h>

			 
/* Include libraries */

#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Iphlpapi.lib")
#pragma comment(lib, "winmm.lib")


/* Defines */

#define MaxPnts 8000									// Maximum number of X/Y points
#define SLEEPTIME 10000									// Process sleep time in microseconds (should be half a Click cycle i.e. 10ms)
#define TADJUST 20000									// The time adjustment divider to get 50 Clicks per second from local MYTIMER() macro function call (see below)
#define SCOPESIZE 512									// Size of the simulator window
#define DEFAULTBACKGROUNDCOLOUR RGB(70,70,110)			// Default background colour for the simulator window

#define MINPPS 1										// Minimum Points Per Second (can't be zero!) (DAC's minimum PPS)

#define IDNMAXCHANNELID 254								// Maximum supported IDN channel ID (must be < 255)
#define IDNMAXWAVEFORMPOINTS 360						// Maximum IDN waveform mode queued frame points
#define IDNQUEUEDTARGETPERIOD 120000					// Queued IDN frame periods target to prevent underruns in microseconds (for LASERfIRE pattern mode and waveform mode) 
#define IDNMINIQUEUEDTARGET 2							// Minimum queued IDN frames target to prevent underruns (for LASERfIRE pattern mode and waveform mode) 
#define IDNMAXIQUEUEDTARGET 6							// Maximum queued IDN frames target to prevent underruns (for LASERfIRE pattern mode and waveform mode) 
#define IDNMAXDACPPS 83000								// DAC's maximum PPS
#define IDNMAXQUEUE 40									// Maximum number of queued IDN frames (must be at least 1)
#define IDNMAXRESYNC 4									// Maximum number of out of order packets (must be at least 1 - too high value might cause underruns)
#define IDNMAXPACKETSIZE 200000							// Maximum packet size in bytes
#define IDNMAXPOINTS MaxPnts							// Max number of IDN points - same as MaxPnts
#define IDNMAXSAMPLESIZE (IDNMAXPOINTS * 20)			// Maximum IDN sample chunk size in bytes
#define IDNQOUTBUFFSIZE (((IDNMAXPOINTS * 15) + 1) / 2)	// Buffer size in bytes for "processed DAC ready" IDN frames (X + Y + r + g + b + ttl nibble = 7.5 bytes/point)
#define IDNBUFFSIZE (IDNMAXSAMPLESIZE + (IDNMAXQUEUE * IDNQOUTBUFFSIZE) + (IDNMAXRESYNC * IDNMAXPACKETSIZE) + IDNQOUTBUFFSIZE + IDNMAXPACKETSIZE)	// IDN total buffer size
#define IDNRESYNCBUFFOFFSET (IDNMAXSAMPLESIZE + (IDNMAXQUEUE * IDNQOUTBUFFSIZE))																	// Offset to start of IDN resync buffer 
#define IDNOUTPUTBUFFOFFSET (IDNMAXSAMPLESIZE + (IDNMAXQUEUE * IDNQOUTBUFFSIZE) + (IDNMAXRESYNC * IDNMAXPACKETSIZE))								// Offset to start of IDN output buffer
#define IDNINPUTBUFFOFFSET (IDNMAXSAMPLESIZE + (IDNMAXQUEUE * IDNQOUTBUFFSIZE) + (IDNMAXRESYNC * IDNMAXPACKETSIZE) + IDNQOUTBUFFSIZE)				// Offset to start of IDN input buffer

#define SIMBUFFSIZE IDNQOUTBUFFSIZE						// Single SIM output buffer size in bytes - should be the same as IDNQOUTBUFFSIZE (gets doubled)
#define SIMPOINTBUFFSIZE sizeof(POINT[IDNMAXPOINTS])	// Size of draw scope polyline POINT array buffer
#define SIMBUFFTOTALSIZE ((2 * SIMBUFFSIZE) + SIMPOINTBUFFSIZE)	// SIM total buffer size

#define DEFAULTIDNPORT 7255								// Default IDN server connection port
#define MINNETWORKPORT 1024								// Minimum network port
#define MAXNETWORKPORT 49151							// Maximum network port
#define TCPTIMEOUTSEC 0									// TCP Connection timeout in seconds
#define TCPTIMEOUTMICRO (SLEEPTIME / 2)					// TCP Connection timeout in microseconds (should be half the process sleep time)
#define IDNUDPREADSIZE IDNMAXPACKETSIZE					// Maximum number of UDP bytes to read in at a time - should be the same as maximum packet size
#define IDNTIMEOUTCLICKS 50								// IDN connection timeout in "clicks" (50 = 1 seconds)
#define IDNTCPMAXTIMEOUTS 100							// Maximum number of IDN TCP timeouts before error (200 = 1 second)


/* IDN defines */

#define IDNCMD_VOID 0x00

#define IDNCMD_SCAN_REQUEST 0x10						// Network scan for units
#define IDNCMD_SCAN_RESPONSE 0x11						// Unit identification and status
#define IDNCMD_SERVICEMAP_REQUEST 0x12					// Request for unit services
#define IDNCMD_SERVICEMAP_RESPONSE 0x13					// Map of supported services

#define IDNCMD_RT_CNLMSG 0x40							// Realtime channel message (empty - keep alive)
#define IDNCMD_RT_CNLMSG_CLOSE 0x44						// Gracefully close (if msg: process, then close)

#define IDNFLG_CONTENTID_CHANNELMSG 0x8000				// Channel message flag (specific bit assignments)
#define IDNFLG_CONTENTID_CONFIG_LSTFRG 0x4000			// Set for config header or last fragment
#define IDNMSK_CONTENTID_CHANNELID 0x3f00				// Channel ID bit mask
#define IDNMSK_CONTENTID_CNKTYPE 0x00ff					// Data chunk type bit mask

#define IDNFLG_CHNCFG_ROUTING 0x01						// Verify/Route/Open channel before message processing
#define IDNFLG_CHNCFG_CLOSE 0x02						// Close channel after message processing

#define IDNVAL_CNKTYPE_VOID 0x00						// Empty chunk (no data)
#define IDNVAL_CNKTYPE_LPGRF_WAVE 0x01					// Sample data array
#define IDNVAL_CNKTYPE_LPGRF_FRAME 0x02					// Sample data array (entirely)
#define IDNVAL_CNKTYPE_LPGRF_FRAME_FIRST 0x03			// Sample data array (first fragment)
#define IDNVAL_CNKTYPE_LPGRF_FRAME_SEQUEL 0xc0			// Sample data array (sequel fragment)

#define IDNVAL_SMOD_VOID 0x00							// No function, no lookup
#define IDNVAL_SMOD_LPGRF_CONTINUOUS 0x01				// Laser graphic: Stream of waveform segments
#define IDNVAL_SMOD_LPGRF_DISCRETE 0x02					// Laser graphic: Stream of individual frames

#define IDNFLG_SCAN_STATUS_REALTIME 0x01				// Offers realtime streaming through IDN-Hello

#define IDNVAL_STYPE_LAPRO 0x80							// Standard laser projector


/* Timer function call macro for local system */

#define MYTIMER() GetTimerTicks_us()					// Windows
#define MYTIMERRETURN ULONGLONG							// Windows


/* Structures */

struct IDNQUEUE
{
	unsigned int PPS;
	unsigned int Points;
	unsigned int Period;
	unsigned int Checksum;
	unsigned int Sequence;
//	unsigned int AppendCount;							// Debug/testing only
};

struct RESPONSEDATA
{
	sockaddr_in *IDNClientAddressPtr;					// Address of the IDN client
	SOCKET *IDNSocketPtr;								// IDN UDP data socket
	SOCKET *ActiveIDNSocketPtr;							// IDN TCP data socket
	unsigned short PacketCount;							// Packet sequence count (UDP packet tracking)
	unsigned char ClientGroup;							// Client group to send on
};

struct LASERFIREDATA
{
	unsigned char *SIMMemPtr;							// Used by Windows SIM() only
	unsigned char *SIMBuffPtr;							// Used by SIM()
	unsigned char *IDNMemPtr;							// Used by IDN mode only
	unsigned char *IDNBuffPtr;							// .
	IDNQUEUE *IDNQueueData;								// .
	RESPONSEDATA *IDNResponseData;						// .
	MYTIMERRETURN LastTime;
	MYTIMERRETURN FrameTimestamp;
	MYTIMERRETURN IDNTimeoutTimestamp;					// Used by IDN mode only
	unsigned int IDNFormat;								// .
	unsigned int IDNLastFormat;							// .
	unsigned int IDNChecksum;							// .
	unsigned int IDNQueueCount;							// .
	unsigned int IDNLastQueueCount;						// .
	unsigned int IDNSpeedAdjust;						// .
	unsigned int IDNLastSpeedAdjust;					// .
	unsigned int IDNQueueTarget;						// .
	unsigned int IDNLastQueueTarget;					// .
	unsigned int SIMPoints;								// Used by SIM()
	unsigned int FramePeriod_us;
	unsigned int SleepError;
	unsigned int LastSleepError;
	unsigned int PPS;
	unsigned int LastPPS;
	unsigned int TotalPoints;
	unsigned int LastTotalPoints;
	unsigned int Freq;
	unsigned int LastFreq;
	unsigned char IDNCurrentChannelID;					// Used by IDN mode only
	bool IDNWaveformMode;								// .
	bool IDNLastWaveformMode;							// .
	bool IDNSyncPacketCountFlag;						// .
	bool IDNGracefullyCloseShutterFlag;					// .
	bool IDNFragmentedFlag;								// .
	bool IDNUnqueueFlag;								// .
	bool SIMBuffSwitch;									// Used by Windows SIM() only
	bool SIMFinalFlag;									// Used by Windows SIM() only
	bool UpdateScope;
	bool ShutterFlag;
	bool LastShutterFlag;
};


/* IDN structures */

struct IDNHDR_PACKET
{
	unsigned char command;								// The command code
	unsigned char flags;								// Upper 4 bits: Flags; Lower 4 bits: Client group
	unsigned short sequence;							// Sequence counter, must count up
};

struct IDNHDR_CHANNEL_MESSAGE
{
	unsigned short totalSize;
	unsigned short contentID;
	unsigned int timestamp;
};

struct IDNHDR_CHANNEL_CONFIG
{
	unsigned char wordCount;
	unsigned char flags;								// Upper 4 bit decoder flags (0x30: Match), lower config
	unsigned char serviceID;
	unsigned char serviceMode;
};

struct IDNHDR_SAMPLE_CHUNK
{
	unsigned int flagsDuration;							// Flags: 0x30: Match
};

struct IDNHDR_SCAN_RESPONSE
{
	unsigned char structSize;							// Size of this struct.
	unsigned char protocolVersion;						// Upper 4 bits: Major; Lower 4 bits: Minor
	unsigned char status;								// Unit and link status flags
	unsigned char reserved;
	unsigned char unitID[16];							// [0]: Len, [1]: Cat, [2..Len]: ID, padded with '\0'
	unsigned char hostName[20];							// Not terminated, padded with '\0'
};

struct IDNHDR_SERVICEMAP_RESPONSE
{
	unsigned char structSize;							// Size of this struct.
	unsigned char entrySize;							// Size of an entry - sizeof(IDNHDR_SERVICEMAP_ENTRY)
	unsigned char relayEntryCount;						// Number of relay entries
	unsigned char serviceEntryCount;					// Number of service entries
};

struct IDNHDR_SERVICEMAP_ENTRY
{
	unsigned char serviceID;							// Service: The ID (!=0); Relay: Must be 0
	unsigned char serviceType;							// The type of the service; Relay: Must be 0
	unsigned char flags;								// Status flags and options
	unsigned char relayNumber;							// Service: Root(0)/Relay(>0); Relay: Number (!=0)
	unsigned char name[20];								// Not terminated, padded with '\0'
};


/* Globals */

unsigned int IDNPort = DEFAULTIDNPORT;									// Configuration settings
bool ShowStatusInfoFlag = false, SIMWindowFlag = true;
bool ShowConnectionInfoFlag = false, ExtendedErrorMsgsFlag = false;
bool DrawBetweenPointsFlag = false, ShowPointsFlag = false, IDNModeFlag = false;

volatile LASERFIREDATA *SIMLfPtr = 0;									// Used by SIM()
volatile unsigned char *SIMBuffPtr = 0;									// Used by SIM()
volatile unsigned int SIMPoints = 0;									// Used by SIM()
volatile int ThreadExitStatus = 0;										// Used by main thread
volatile bool SIMReady = true;											// Used by SIM()
volatile bool ExitFlag = false;											// Used by SIM() and main thread
volatile HWND SIMWindow = 0;											// Used by SIM()

unsigned int ErrorCode = 0;												// The global error code


/* Functions - in compile order */

unsigned int GetNum(unsigned char **StrStartPtrPtr, unsigned char *StrEndPtr)	// Default is 0
{
	unsigned char *StrPtr = *StrStartPtrPtr;
	int i, n = 0;

	while (StrPtr < StrEndPtr && (*StrPtr == ' ' || *StrPtr == '\t'))	// Skip white space
		StrPtr++;

	for (i = 0; i < 10 && StrPtr < StrEndPtr && *StrPtr >= '0' && *StrPtr <= '9'; i++)
		n = n * 10 + (*StrPtr++ - '0');

	*StrStartPtrPtr = StrPtr;
	return n;
}


void PrintErrorCodeString(void)
{
	char ErrorString[1024];												// Formatted system error msgs are quite large

	if (ErrorCode != 0){
		if (FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, 0, ErrorCode, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), ErrorString, sizeof(ErrorString), 0) != 0){
			printf("Last system error: %s\n", ErrorString);
			ErrorCode = 0;
			errno = 0;
		}

	} else {

		if (errno != 0){
			strerror_s(ErrorString, sizeof(ErrorString), errno);
			printf("Last system error: \"%s\"\n\n", ErrorString);
			errno = 0;
		} else {
			printf("\n");
		}
	}

	return;
}


ULONGLONG GetTimerTicks_us(void)										// No "Fall back" version
{
	static bool First = true;
	static LARGE_INTEGER Freq;
	LARGE_INTEGER Ticks;

	if (First == true){													// First call?

		if (QueryPerformanceFrequency(&Freq) == 0)						// Get preformance counter frequency (counts per second)
			return 0;

		First = false;													// Only once
	}

	if (QueryPerformanceCounter(&Ticks) == 0)							// Get current ticks
		return 0;

	return ((Ticks.QuadPart * 1000000) / Freq.QuadPart);
}


unsigned char *MakeAddressStr(unsigned char *InAddPtr)
{
	static unsigned char AddressStr[16];
	
	if (inet_ntop(AF_INET, InAddPtr, (char *)AddressStr, sizeof(AddressStr)) == 0){
		AddressStr[0] = '?';
		AddressStr[1] = '?';
		AddressStr[2] = '?';
		AddressStr[3] = '.';
		AddressStr[4] = '?';
		AddressStr[5] = '?';
		AddressStr[6] = '?';
		AddressStr[7] = '.';
		AddressStr[8] = '?';
		AddressStr[9] = '?';
		AddressStr[10] = '?';
		AddressStr[11] = '.';
		AddressStr[12] = '?';
		AddressStr[13] = '?';
		AddressStr[14] = '?';
		AddressStr[15] = 0;
	}

	return AddressStr;
}


void ShowAddresses(char *MsgPtr, unsigned int Port)
{
	unsigned int AddressLen;
	ULONG r;
	sockaddr_in *Address;
	IP_ADAPTER_ADDRESSES *TempPtr;
	IP_ADAPTER_UNICAST_ADDRESS *UnicastPtr;
	IP_ADAPTER_ADDRESSES AdapterAddresses[20];
	
	AddressLen = sizeof(AdapterAddresses);

	if ((r = GetAdaptersAddresses(AF_INET, 0, 0, AdapterAddresses, (PULONG)&AddressLen)) != NO_ERROR){
		printf("%sCan't get network adapters addresses! (e=%d)\n", MsgPtr, r);
		return;
	}

	TempPtr = AdapterAddresses;

	while (TempPtr){

		UnicastPtr = TempPtr->FirstUnicastAddress;

		while (UnicastPtr){
				
			if (UnicastPtr->Address.lpSockaddr->sa_family == AF_INET){
				Address = (sockaddr_in *)UnicastPtr->Address.lpSockaddr;
				printf("%s\033[1;33m%s\033[0m:\033[1;36m%d\033[0m (\"%wS\").\n", MsgPtr, MakeAddressStr((unsigned char *)&Address->sin_addr), Port, TempPtr->FriendlyName);
			}

			UnicastPtr = UnicastPtr->Next;
		}

		TempPtr = TempPtr->Next;
	}

	return;
}


SOCKET InitIDNServ(void)
{
	static sockaddr_in IDNAddress;
	unsigned char *TempPtr;
	int Temp;
	unsigned int i;
	BOOL True = TRUE;
	u_long Mode = 1;
	SOCKET IDNSocket;
	WSADATA Crap;
	
	if ((ErrorCode = WSAStartup(MAKEWORD(2, 2), &Crap)) != 0){			// Request WinSock version 2.2
		printf("Can't initialise WinSock 2.2! (e=%d)\n", ErrorCode);
		return 0;
	}

	TempPtr = (unsigned char *)&IDNAddress;								// Zero out address structure
	for (i = sizeof(IDNAddress); i != 0 ; i--)
	  *TempPtr++ = 0;

    IDNAddress.sin_family = AF_INET;
	IDNAddress.sin_addr.s_addr = INADDR_ANY;
	IDNAddress.sin_port = htons(IDNPort);								// Set port

	if ((IDNSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET){	// "UDP" protocol
		ErrorCode = WSAGetLastError();
		printf("Can't create IDN data socket! (e=%d)\n", ErrorCode);
		return 0;
	}

	if (ioctlsocket(IDNSocket, FIONBIO, &Mode) == SOCKET_ERROR){		// Set socket to non-blocking
		ErrorCode = WSAGetLastError();
		printf("Can't ioctlsocket() on IDN data socket! (e=%d)\n", ErrorCode);
		Temp = errno;
		closesocket(IDNSocket);
		errno = Temp;
		return 0;
	}

	if (bind(IDNSocket, (sockaddr *)&IDNAddress, sizeof(IDNAddress)) == SOCKET_ERROR){
		ErrorCode = WSAGetLastError();
		printf("Can't bind IDN data socket! (e=%d)\n", ErrorCode);
		Temp = errno;
		closesocket(IDNSocket);
		errno = Temp;
		return 0;
	}

	ShowAddresses((char *)"Assigned IDN UDP data socket: ", IDNPort);
	return IDNSocket;
}


SOCKET InitIDNTCPServ(void)
{
	static sockaddr_in IDNAddress;
	unsigned char *TempPtr;
	int Temp;
	unsigned int i;
	BOOL True = TRUE;
	u_long Mode = 1;
	SOCKET IDNSocket;
	
	/* Assumes WinSock already initalised */
		
	TempPtr = (unsigned char *)&IDNAddress;								// Zero out address structure
	for (i = sizeof(IDNAddress); i != 0 ; i--)
	  *TempPtr++ = 0;

    IDNAddress.sin_family = AF_INET;
	IDNAddress.sin_addr.s_addr = INADDR_ANY;
	IDNAddress.sin_port = htons(IDNPort);								// Set port

	if ((IDNSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == INVALID_SOCKET){	// "TCP" protocol
		ErrorCode = WSAGetLastError();
		printf("Can't create IDN TCP data socket! (e=%d)\n", ErrorCode);
		Temp = errno;
		WSACleanup(); 
		errno = Temp;
		return 0;
	}

	if (setsockopt(IDNSocket, SOL_SOCKET, SO_REUSEADDR, (char *)&True, sizeof(True)) == SOCKET_ERROR){
		ErrorCode = WSAGetLastError();
		printf("Can't set IDN TCP data socket options! (e=%d)\n", ErrorCode);
		Temp = errno;
		closesocket(IDNSocket);
		WSACleanup();
		errno = Temp;
		return 0;
	}

	if (ioctlsocket(IDNSocket, FIONBIO, &Mode) == SOCKET_ERROR){		// Set socket to non-blocking
		ErrorCode = WSAGetLastError();
		printf("Can't ioctlsocket() on IDN TCP data socket! (e=%d)\n", ErrorCode);
		Temp = errno;
		closesocket(IDNSocket);
		WSACleanup();
		errno = Temp;
		return 0;
	}

	if (bind(IDNSocket, (sockaddr *)&IDNAddress, sizeof(IDNAddress)) == SOCKET_ERROR){
		ErrorCode = WSAGetLastError();
		printf("Can't bind IDN TCP data socket! (e=%d)\n", ErrorCode);
		Temp = errno;
		closesocket(IDNSocket);
		WSACleanup();
		errno = Temp;
		return 0;
	}

	if (listen(IDNSocket, 1) == SOCKET_ERROR){
		ErrorCode = WSAGetLastError();
		printf("Can't set IDN TCP data socket for listen! (e=%d)\n", ErrorCode);
		Temp = errno;
		closesocket(IDNSocket);
		WSACleanup();
		errno = Temp;
		return 0;
	}

	ShowAddresses((char *)"Assigned IDN TCP data socket: ", IDNPort);
	return IDNSocket;
}


int CloseLastSocket(SOCKET *LastActiveSocketPtr)
{
	if (*LastActiveSocketPtr != 0){										// Close previous socket?

		shutdown(*LastActiveSocketPtr, SD_BOTH);

		if (closesocket(*LastActiveSocketPtr) == SOCKET_ERROR){
			ErrorCode = WSAGetLastError();
			printf("Error while closing last active socket! (e=%d)\n", ErrorCode);
			PrintErrorCodeString();
			*LastActiveSocketPtr = 0;
			return -1;
		}

		*LastActiveSocketPtr = 0;
	}

	return 0;
}


int GetIDNData(SOCKET IDNSocket, sockaddr_in *IDNClientAddressPtr, unsigned char *BuffPtr)
{
	int IDNAddressLen, r;
	fd_set SetsRd;
	timeval Time;

	FD_ZERO(&SetsRd);													// Check for IDN data
	FD_SET(IDNSocket, &SetsRd);
	Time.tv_sec = 0;
	Time.tv_usec = 0;
	r = select(0, &SetsRd, 0, 0, &Time);
		
	if (r == 0)															// Any data?
		return 0;

	if (r != 1){
		ErrorCode = WSAGetLastError();
		printf("Error while polling IDN UDP data socket! (e=%d)\n", ErrorCode);
		return -1;
	}
	
	IDNAddressLen = sizeof(sockaddr_in);

	if ((r = recvfrom(IDNSocket, (char *)BuffPtr, IDNUDPREADSIZE, 0, (sockaddr *)IDNClientAddressPtr, &IDNAddressLen)) == SOCKET_ERROR){
		ErrorCode = WSAGetLastError();
		printf("Error while receiving data on IDN UDP socket! (e=%d)\n", ErrorCode);
		return -1;
	}

	if (r == 0)															// Just in case															
		return 0;

	if (ShowConnectionInfoFlag == true)
		printf("Received %d IDN UDP data bytes from %s:%d\n", r, MakeAddressStr((unsigned char *)&IDNClientAddressPtr->sin_addr), ntohs(IDNClientAddressPtr->sin_port));

	return r;
}


int GetIDNTCPData(SOCKET IDNTCPSocket, SOCKET *ActiveSocketPtr, sockaddr_in *IDNClientAddressPtr, unsigned char *BuffPtr, bool *NoSleepFlagPtr)
{
	static unsigned int Size, TimeOutCount;
	static unsigned short TotalSize = 0;
	int IDNAddressLen, r;
	fd_set SetsRd;
	timeval Time;
	IDNHDR_PACKET *PacketHeaderPtr;	
	IDNHDR_CHANNEL_MESSAGE *ChannelMsgHeaderPtr;

	if (*ActiveSocketPtr == 0){											// Active connection yet?

		FD_ZERO(&SetsRd);												// Check for connection
		FD_SET(IDNTCPSocket, &SetsRd);
		Time.tv_sec = 0;
		Time.tv_usec = 0;
		r = select(0, &SetsRd, 0, 0, &Time);
		
		if (r == 0)														// Any data?
			return 0;

		if (r != 1){
			ErrorCode = WSAGetLastError();
			printf("Error while polling IDN TCP data socket! (e=%d)\n", ErrorCode);
			return -1;
		}

		IDNAddressLen = sizeof(sockaddr_in);

		if ((*ActiveSocketPtr = accept(IDNTCPSocket, (sockaddr *)IDNClientAddressPtr, &IDNAddressLen)) == INVALID_SOCKET){
			ErrorCode = WSAGetLastError();
			printf("Error while accepting connection from IDN TCP client! (e=%d)\n", ErrorCode);
			*ActiveSocketPtr = 0;
			return -1;
		}

		if (ShowConnectionInfoFlag == true)
			printf("Received IDN TCP connection from %s:%d\n", MakeAddressStr((unsigned char *)&IDNClientAddressPtr->sin_addr), ntohs(IDNClientAddressPtr->sin_port));
	}

	if (TotalSize == 0){

		FD_ZERO(&SetsRd);
		FD_SET(*ActiveSocketPtr, &SetsRd);
		Time.tv_sec = 0;
		Time.tv_usec = 0;
		r = select(0, &SetsRd, 0, 0, &Time);
	
		if (r == 0)														// Any data?
			return 0;

		if (r != 1){
			ErrorCode = WSAGetLastError();
			printf("Error while polling active IDN TCP socket! (e=%d)\n", ErrorCode);
			r = errno;
			shutdown(*ActiveSocketPtr, SD_BOTH);
			closesocket(*ActiveSocketPtr);
			errno = r;
			*ActiveSocketPtr = 0;
			return -1;
		}

		if ((r = recv(*ActiveSocketPtr, (char *)BuffPtr, (sizeof(IDNHDR_PACKET) + sizeof(IDNHDR_CHANNEL_MESSAGE)), 0)) == SOCKET_ERROR){	// Receive data
			ErrorCode = WSAGetLastError();
			printf("Error while reading data on active IDN TCP socket! (e=%d)\n", ErrorCode);
			r = errno;
			shutdown(*ActiveSocketPtr, SD_BOTH);
			closesocket(*ActiveSocketPtr);
			errno = r;
			*ActiveSocketPtr = 0;
			return -1;
		}
	
		if (r == 0){													// Socket has been Shutdown/closed by the other side
			shutdown(*ActiveSocketPtr, SD_BOTH);
			closesocket(*ActiveSocketPtr);
			*ActiveSocketPtr = 0;
			if (ShowConnectionInfoFlag == true)
				printf("The IDN TCP connection from %s:%d was actively closed.\n", MakeAddressStr((unsigned char *)&IDNClientAddressPtr->sin_addr), ntohs(IDNClientAddressPtr->sin_port));
			return 0;
		}

		if (r != (sizeof(IDNHDR_PACKET) + sizeof(IDNHDR_CHANNEL_MESSAGE))){	// Got channel message?
			Size = r;
			goto Done;
		}

		PacketHeaderPtr = (IDNHDR_PACKET *)BuffPtr;								// Point to packet header
		ChannelMsgHeaderPtr = (IDNHDR_CHANNEL_MESSAGE *)&PacketHeaderPtr[1];	// Point to message header
		TotalSize = ntohs(ChannelMsgHeaderPtr->totalSize);						// Get TotalSize

		if (TotalSize <= sizeof(IDNHDR_CHANNEL_MESSAGE)){				// Just a channel message?
			Size = r;
			goto Done;
		}

		if ((TotalSize + sizeof(IDNHDR_PACKET)) > IDNMAXPACKETSIZE){
			if (ExtendedErrorMsgsFlag == true)
				printf("Warning, IDN message size exceeds input buffer on incoming TCP data! (%d>%d)\n", (int)(TotalSize + sizeof(IDNHDR_PACKET)), IDNMAXPACKETSIZE);
			shutdown(*ActiveSocketPtr, SD_BOTH);						// Force new TCP connection
			closesocket(*ActiveSocketPtr);
			*ActiveSocketPtr = 0;
			TotalSize = 0;
			return 0;
		}

		Size = 0;
		TimeOutCount = 0;
	}

	do {

		FD_ZERO(&SetsRd);												// Make sure there's something to read
		FD_SET(*ActiveSocketPtr, &SetsRd);
		Time.tv_sec = TCPTIMEOUTSEC;
		Time.tv_usec = TCPTIMEOUTMICRO;
		r = select(0, &SetsRd, 0, 0, &Time);
	
		if (r == 0){													// Timeout?

			if (++TimeOutCount == IDNTCPMAXTIMEOUTS){
				if (ExtendedErrorMsgsFlag == true)
					printf("Timeout while waiting for IDN TCP data! (c=%d/%d)\n", Size, (int)(TotalSize - sizeof(IDNHDR_CHANNEL_MESSAGE)));
				shutdown(*ActiveSocketPtr, SD_BOTH);					// Force new TCP connection
				closesocket(*ActiveSocketPtr);
				*ActiveSocketPtr = 0;
				TotalSize = 0;
				return 0;
			}

			*NoSleepFlagPtr = true;										// Force no sleep cycle because of timeout wait time
			return 0;
		}

		if (r != 1){
			ErrorCode = WSAGetLastError();
			printf("Error while polling active IDN TCP socket! (e=%d)\n", ErrorCode);
			r = errno;
			shutdown(*ActiveSocketPtr, SD_BOTH);
			closesocket(*ActiveSocketPtr);
			errno = r;
			*ActiveSocketPtr = 0;
			TotalSize = 0;
			return -1;
		}

		TimeOutCount = 0;

		if ((r = recv(*ActiveSocketPtr, (char *)(BuffPtr + sizeof(IDNHDR_PACKET) + sizeof(IDNHDR_CHANNEL_MESSAGE) + Size), ((TotalSize - sizeof(IDNHDR_CHANNEL_MESSAGE) - Size)), 0)) == SOCKET_ERROR){	// Receive data
			ErrorCode = WSAGetLastError();
			printf("Error while reading data on active IDN TCP socket! (e=%d)\n", ErrorCode);
			r = errno;
			shutdown(*ActiveSocketPtr, SD_BOTH);
			closesocket(*ActiveSocketPtr);
			errno = r;
			*ActiveSocketPtr = 0;
			TotalSize = 0;
			return -1;
		}

		Size += r;

	} while (Size < (TotalSize - sizeof(IDNHDR_CHANNEL_MESSAGE)));

	Size += (sizeof(IDNHDR_PACKET) + sizeof(IDNHDR_CHANNEL_MESSAGE));

Done:
	
	TotalSize = 0;

	if (ShowConnectionInfoFlag == true)
		printf("Received %d IDN TCP data bytes from %s:%d\n", Size, MakeAddressStr((unsigned char *)&IDNClientAddressPtr->sin_addr), ntohs(IDNClientAddressPtr->sin_port));

	return Size;
}


bool SendIDNPacket(RESPONSEDATA *DataPtr, unsigned char *PacketPtr, unsigned int Size)
{
	unsigned int Count = 0, r;
	
	if (Size == 0)
		return true;

	if (*DataPtr->ActiveIDNSocketPtr == 0){								// TCP?
	
		do {

			if ((r = sendto(*DataPtr->IDNSocketPtr, (char *)(PacketPtr + Count), (Size - Count), 0, (sockaddr *)DataPtr->IDNClientAddressPtr, sizeof(sockaddr_in))) == SOCKET_ERROR){
				ErrorCode = WSAGetLastError();
				printf("Error while sending IDN response on UDP data socket! (e=%d)\n", ErrorCode);
				return false;
			}
			
			Count += r;

		} while (Count < Size);

		if (ShowConnectionInfoFlag == true)
			printf("Sent %d IDN UDP data bytes to %s:%d\n", Size, MakeAddressStr((unsigned char *)&DataPtr->IDNClientAddressPtr->sin_addr), ntohs(DataPtr->IDNClientAddressPtr->sin_port));

	} else {

		do {

			if ((r = send(*DataPtr->ActiveIDNSocketPtr, (char *)(PacketPtr + Count), (Size - Count), 0)) == SOCKET_ERROR){
				ErrorCode = WSAGetLastError();
				printf("Error while sending IDN response on TCP data socket! (e=%d)\n", ErrorCode);
				return false;
			}

			Count += r;

		} while (Count < Size);

		if (ShowConnectionInfoFlag == true)
			printf("Sent %d IDN TCP data bytes to %s:%d\n", Size, MakeAddressStr((unsigned char *)&DataPtr->IDNClientAddressPtr->sin_addr), ntohs(DataPtr->IDNClientAddressPtr->sin_port));
	}

	return true;
}


bool IDNSendScanResponse(RESPONSEDATA *DataPtr)
{
	unsigned char *TempPtr;
	unsigned int i;
	unsigned char Buff[sizeof(IDNHDR_PACKET) + sizeof(IDNHDR_SCAN_RESPONSE)];
	IDNHDR_PACKET *PacketHeaderPtr = (IDNHDR_PACKET *)Buff;													// Setup pointers
	IDNHDR_SCAN_RESPONSE *ScanResponseHeaderPtr = (IDNHDR_SCAN_RESPONSE *)&PacketHeaderPtr[1];				// .

	TempPtr = Buff;																							// Zero out buffer
	for (i = sizeof(Buff); i != 0; i--)
		*TempPtr++ = 0;

	PacketHeaderPtr->command = IDNCMD_SCAN_RESPONSE;														// (1) "hello" header (Unit identification and status)
	PacketHeaderPtr->flags = DataPtr->ClientGroup;															// Set ClientGroup
	PacketHeaderPtr->sequence = htons(DataPtr->PacketCount);												// Set packet sequence count

	ScanResponseHeaderPtr->structSize = sizeof(IDNHDR_SCAN_RESPONSE);										// (2) Set size
	ScanResponseHeaderPtr->protocolVersion = 1;																// Toolbox sets this as 1
	ScanResponseHeaderPtr->status = IDNFLG_SCAN_STATUS_REALTIME;											// Set status (Offers realtime streaming through IDN-Hello)
	ScanResponseHeaderPtr->unitID[0] = 7;																	// (Len) - Toolbox sets this as 7
	ScanResponseHeaderPtr->unitID[1] = 1;																	// (Cat) - Toolbox sets this as 1

	ScanResponseHeaderPtr->hostName[0] = 'I';																// Hostname
	ScanResponseHeaderPtr->hostName[1] = 'D';																// .
	ScanResponseHeaderPtr->hostName[2] = 'N';																// .
	ScanResponseHeaderPtr->hostName[3] = '-';																// .
	ScanResponseHeaderPtr->hostName[4] = 'S';																// .
	ScanResponseHeaderPtr->hostName[5] = 'c';																// .
	ScanResponseHeaderPtr->hostName[6] = 'o';																// .
	ScanResponseHeaderPtr->hostName[7] = 'p';																// .
	ScanResponseHeaderPtr->hostName[8] = 'e';																// .

	return SendIDNPacket(DataPtr, Buff, sizeof(Buff));														// Send response packet
}


bool IDNSendServiceMapResponse(RESPONSEDATA *DataPtr)
{
	unsigned char *TempPtr;
	unsigned int i;
	unsigned char Buff[sizeof(IDNHDR_PACKET) + sizeof(IDNHDR_SERVICEMAP_RESPONSE) + sizeof(IDNHDR_SERVICEMAP_ENTRY)];
	IDNHDR_PACKET *PacketHeaderPtr = (IDNHDR_PACKET *)Buff;															// Setup pointers
	IDNHDR_SERVICEMAP_RESPONSE *ServiceMapResponseHeaderPtr = (IDNHDR_SERVICEMAP_RESPONSE *)&PacketHeaderPtr[1];	// .
	IDNHDR_SERVICEMAP_ENTRY *ServiceMapEntryHeaderPtr = (IDNHDR_SERVICEMAP_ENTRY *)&ServiceMapResponseHeaderPtr[1];	// .

	TempPtr = Buff;																							// Zero out buffer
	for (i = sizeof(Buff); i != 0; i--)
		*TempPtr++ = 0;

	PacketHeaderPtr->command = IDNCMD_SERVICEMAP_RESPONSE;													// (1) "hello" header (Unit identification and status)
	PacketHeaderPtr->flags = DataPtr->ClientGroup;															// Set ClientGroup
	PacketHeaderPtr->sequence = htons(DataPtr->PacketCount);												// Set packet sequence count

	ServiceMapResponseHeaderPtr->structSize = sizeof(IDNHDR_SERVICEMAP_RESPONSE);							// (2) Set size
	ServiceMapResponseHeaderPtr->entrySize = sizeof(IDNHDR_SERVICEMAP_ENTRY);								// Set size
//	ServiceMapResponseHeaderPtr->relayEntryCount = 0;														// Toolbox sets this as 0
	ServiceMapResponseHeaderPtr->serviceEntryCount = 1;														// Set number of service entires

	ServiceMapEntryHeaderPtr->serviceID = 1;																// (3) Set Serivce ID (can't be 0)
	ServiceMapEntryHeaderPtr->serviceType = IDNVAL_STYPE_LAPRO;												// Set service type (Standard laser projector)
//	ServiceMapEntryHeaderPtr->flags = 0;																	// Toolbox sets this as 0
//	ServiceMapEntryHeaderPtr->relayNumber = 0;																// Toolbox sets this as 0
	ServiceMapEntryHeaderPtr->name[0] = 'S';																// Set service name
	ServiceMapEntryHeaderPtr->name[1] = 'i';																// .
	ServiceMapEntryHeaderPtr->name[2] = 'm';																// .
	ServiceMapEntryHeaderPtr->name[3] = 'u';																// .
	ServiceMapEntryHeaderPtr->name[4] = 'l';																// .
	ServiceMapEntryHeaderPtr->name[5] = 'a';																// .
	ServiceMapEntryHeaderPtr->name[6] = 't';																// .
	ServiceMapEntryHeaderPtr->name[7] = 'o';																// .
	ServiceMapEntryHeaderPtr->name[8] = 'r';																// .
	ServiceMapEntryHeaderPtr->name[9] = '_';																// .
	ServiceMapEntryHeaderPtr->name[10] = 'o';																// .
	ServiceMapEntryHeaderPtr->name[11] = 'n';																// .
	ServiceMapEntryHeaderPtr->name[12] = 'l';																// .
	ServiceMapEntryHeaderPtr->name[13] = 'y';																// .

	return SendIDNPacket(DataPtr, Buff, sizeof(Buff));														// Send response packet
}


unsigned int MakeIDNDACData(unsigned char *DataPtr, unsigned short *DestPtr, unsigned int Points, unsigned int Format, unsigned int LastPoints, unsigned int Checksum)
{
	unsigned char *ChecksumPtr, *TTLDestPtr, *ColourDestPtr;
	unsigned int i = 0;
	short X, Y;
	unsigned char TTL = 0, nibble, c;

	if (LastPoints != 0){												// Append new points to existing DAC buffer?
		DestPtr += (LastPoints * 2);									// Point to start of new YX data
		ChecksumPtr = (unsigned char *)DestPtr;
		memmove((ChecksumPtr + (Points * 4)), ChecksumPtr, (((LastPoints * 7) + 1) / 2));	// Move existing Colour and TTL data up by new colour offset (Size = Points x 3.5 rounded up)
		ColourDestPtr = (ChecksumPtr + (Points * 4) + (LastPoints * 3));
		memmove((ColourDestPtr + (Points * 3)), ColourDestPtr, ((LastPoints + 1) / 2));		// Move existing TTL data up (Size = Points x 0.5 rounded up)
		TTLDestPtr = (ColourDestPtr + (Points * 3) + ((LastPoints + 1) / 2));
		if (LastPoints & 1){											// "Odd" number of last TTL nibbles?
			--TTLDestPtr;												// Point to "old" incomplete TTL byte
			Checksum -= TTL = *TTLDestPtr;								// Get previous incomplete TTL byte and update checksum
			TTL = (TTL << 4);											// Restore nibble alignment
			i = 1;														// Offset start to "old" number
			Points++;													// Offset point counter
		}
	} else {
		ChecksumPtr = (unsigned char *)DestPtr;
		ColourDestPtr = (ChecksumPtr + (Points * 4));
		TTLDestPtr = (ChecksumPtr + (Points * 7));
	}

	/* Make TTL and 24 bit colour DAC data -> Y-X-r-g-b-TTL_nibble (7.5 bytes / point) */

	switch (Format){

	case 7:																// Format = 7 bytes: 16 bit XY + 8 bit rgb

		for (; i < Points ; i++){
			X = *DataPtr++ << 8;										// HIGH(X)
			X |= *DataPtr++;											// LOW(X)
			Y = *DataPtr++ << 8;										// HIGH(Y)
			Y |= *DataPtr++;											// LOW(Y)

			*DestPtr++ = (Y + 32768);									// Y + offset
			Checksum += *ChecksumPtr++;									// Add checksum of Y bytes
			Checksum += *ChecksumPtr++;
			*DestPtr++ = (X + 32768);									// X + offset
			Checksum += *ChecksumPtr++;									// Add checksum of X bytes
			Checksum += *ChecksumPtr++;

			nibble = 0;
			if ((c = *DataPtr++))										// r
				nibble |= 0xc0;
			Checksum += *ColourDestPtr++ = c;							// r
			if ((c = *DataPtr++))										// g
				nibble |= 0xa0;
			Checksum += *ColourDestPtr++ = c;							// g
			if ((c = *DataPtr++))										// b
				nibble |= 0x90;
			Checksum += *ColourDestPtr++ = c;							// b
				
			TTL = ((TTL >> 4) | nibble);								// TTL nibble

			if (i & 1)													// Got two TTL nibbles?
				Checksum += *TTLDestPtr++ = TTL;						// Append TTL colour byte
		}

		break;
	
	case 8:																// Format = 8 bytes: 16 bit XY + 8 bit rgbi

		for (; i < Points ; i++){
			X = *DataPtr++ << 8;										// HIGH(X)
			X |= *DataPtr++;											// LOW(X)
			Y = *DataPtr++ << 8;										// HIGH(Y)
			Y |= *DataPtr++;											// LOW(Y)

			*DestPtr++ = (Y + 32768);									// Y + offset
			Checksum += *ChecksumPtr++;									// Add checksum of Y bytes
			Checksum += *ChecksumPtr++;
			*DestPtr++ = (X + 32768);									// X + offset
			Checksum += *ChecksumPtr++;									// Add checksum of X bytes
			Checksum += *ChecksumPtr++;

			nibble = 0;
			if ((c = *DataPtr++))										// r
				nibble |= 0xc0;
			Checksum += *ColourDestPtr++ = c;							// r
			if ((c = *DataPtr++))										// g
				nibble |= 0xa0;
			Checksum += *ColourDestPtr++ = c;							// g
			if ((c = *DataPtr++))										// b
				nibble |= 0x90;
			Checksum += *ColourDestPtr++ = c;							// b
			DataPtr++;													// Skip i (intensity - ToDo?)
				
			TTL = ((TTL >> 4) | nibble);								// TTL nibble

			if (i & 1)													// Got two TTL nibbles?
				Checksum += *TTLDestPtr++ = TTL;						// Append TTL colour byte
		}

		break;
	
	case 10:															// Format = 10 bytes: 16 bit XYRGB
					
		for (; i < Points ; i++){
			X = *DataPtr++ << 8;										// HIGH(X)
			X |= *DataPtr++;											// LOW(X)
			Y = *DataPtr++ << 8;										// HIGH(Y)
			Y |= *DataPtr++;											// LOW(Y)

			*DestPtr++ = (Y + 32768);									// Y + offset
			Checksum += *ChecksumPtr++;									// Add checksum of Y bytes
			Checksum += *ChecksumPtr++;
			*DestPtr++ = (X + 32768);									// X + offset
			Checksum += *ChecksumPtr++;									// Add checksum of X bytes
			Checksum += *ChecksumPtr++;

			nibble = 0;
			if ((c = *DataPtr++))										// HIGH(R)
				nibble |= 0xc0;
			Checksum += *ColourDestPtr++ = c;							// r
			DataPtr++;													// Skip LOW(R)
			if ((c = *DataPtr++))										// HIGH(G)
				nibble |= 0xa0;
			Checksum += *ColourDestPtr++ = c;							// g
			DataPtr++;													// Skip LOW(G)
			if ((c = *DataPtr++))										// HIGH(B)
				nibble |= 0x90;
			Checksum += *ColourDestPtr++ = c;							// b
			DataPtr++;													// Skip LOW(B)
				
			TTL = ((TTL >> 4) | nibble);								// TTL nibble

			if (i & 1)													// Got two TTL nibbles?
				Checksum += *TTLDestPtr++ = TTL;						// Append TTL colour byte
		}

		break;
	
	case 20:															// Format = 20 bytes: 16 bit XYRGBI1234
					
		for (; i < Points ; i++){
			X = *DataPtr++ << 8;										// HIGH(X)
			X |= *DataPtr++;											// LOW(X)
			Y = *DataPtr++ << 8;										// HIGH(Y)
			Y |= *DataPtr++;											// LOW(Y)

			*DestPtr++ = (Y + 32768);									// Y + offset
			Checksum += *ChecksumPtr++;									// Add checksum of Y bytes
			Checksum += *ChecksumPtr++;
			*DestPtr++ = (X + 32768);									// X + offset
			Checksum += *ChecksumPtr++;									// Add checksum of X bytes
			Checksum += *ChecksumPtr++;

			nibble = 0;
			if ((c = *DataPtr++))										// HIGH(R)
				nibble |= 0xc0;
			Checksum += *ColourDestPtr++ = c;							// r
			DataPtr++;													// Skip LOW(R)
			if ((c = *DataPtr++))										// HIGH(G)
				nibble |= 0xa0;
			Checksum += *ColourDestPtr++ = c;							// g
			DataPtr++;													// Skip LOW(G)
			if ((c = *DataPtr))											// HIGH(B)
				nibble |= 0x90;
			Checksum += *ColourDestPtr++ = c;							// b
			DataPtr += 12;												// Skip LOW(B) & 16 bit I1234 (Intensity - ToDo?)
				
			TTL = ((TTL >> 4) | nibble);								// TTL nibble

			if (i & 1)													// Got two TTL nibbles?
				Checksum += *TTLDestPtr++ = TTL;						// Append TTL colour byte
		}

		break;

	/* Add more formats here */

	default:
					
		printf("MakeIDNDACData(): Unexpected format! (format=%d)\n", Format);	// Should never get here!	
		return 0;
	}

	/* Append final TTL byte - if any (all formats) */

	if (i & 1)															// Odd number of TTL nibbles?
		Checksum += *TTLDestPtr = (TTL >> 4);							// Append final TTL nibble byte
	
	return Checksum;
}


unsigned int CalcFreq(unsigned int PPS, unsigned int Points)
{
	unsigned int Freq;

	if ((Freq = (PPS * 10) / Points) == 0)
		return 1;

	return Freq;
}


void ProcessIDNData(LASERFIREDATA *DataPtr, unsigned int Size)
{
	static unsigned char *DataEOFPtr;
	static unsigned long long TrueLastTimestamp_us;						// Same as ULONGLONG (64 bit)
	static unsigned int ResyncSize[IDNMAXRESYNC];
	static unsigned int ErrorCount = 0, LostPacketCount = 0, SpeedAdjustRestoreCount = 0, LastTimestamp_us, Period;
	static unsigned short NextPacketCount = 0;
	char *TempPtr = 0;
	unsigned char *DataChunkStartPtr;
	unsigned short *DescriptorsPtr;
	unsigned int Timestamp_us, Points, PPS, i, Temp;
	unsigned short PacketCount, TotalSize, ContentID, PacketCountTemp;
	unsigned char PacketCommand, ChannelID, ChunkType, CfgWordCount, CfgFlags, CfgServiceMode;
//	unsigned char PacketFlags, CfgServiceID;							// Not used (yet)
	IDNHDR_PACKET *PacketHeaderPtr, *PacketHeaderTempPtr;	
	IDNHDR_CHANNEL_MESSAGE *ChannelMsgHeaderPtr;
	IDNHDR_CHANNEL_CONFIG *ChannelConfigHeaderPtr;
	IDNHDR_SAMPLE_CHUNK *SampleChunkHeaderPtr;

	if (Size < sizeof(IDNHDR_PACKET)){									// Check minimum data size
		if (ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Received packet data is too small (size=%d)!\n", ++ErrorCount, Size);
		DataPtr->IDNTimeoutTimestamp = (MYTIMER() + ((IDNTIMEOUTCLICKS / 2) * TADJUST));	// Reduced IDN connection timeout timestamp 
		goto Error;
	}

	DataPtr->IDNTimeoutTimestamp = (MYTIMER() + (IDNTIMEOUTCLICKS * TADJUST));		// Update IDN connection timeout timestamp 

	PacketHeaderPtr = (IDNHDR_PACKET *)(DataPtr->IDNMemPtr + IDNINPUTBUFFOFFSET);	// Point to packet header
	PacketCount = ntohs(PacketHeaderPtr->sequence);						// Get PacketCount (ntohs() - because of byte order)
	PacketCommand = PacketHeaderPtr->command;							// Get PacketCommand (single byte)
//	PacketFlags = PacketHeaderPtr->flags;								// Get PacketFlags (single byte)

	switch (PacketCommand){

	case IDNCMD_VOID:
		DataPtr->IDNSyncPacketCountFlag = true;							// Resync packet count
		DataPtr->IDNFragmentedFlag = false;								// Reset fragment flag
		return;															// Ignore

	case IDNCMD_RT_CNLMSG:												// RT Channel message follows?
		break;

	case IDNCMD_RT_CNLMSG_CLOSE:										// RT Channel message close?

		DataPtr->IDNFormat = 7;											// Set format to default (7 bytes)
		DataPtr->IDNCurrentChannelID = 0xff;							// Set channel closed
		DataPtr->IDNWaveformMode = false;								// Set mode to default
		DataPtr->IDNGracefullyCloseShutterFlag = true;					// Signal main loop to close shutter when last frame sent
		DataPtr->IDNSyncPacketCountFlag = true;							// Resync packet count
		DataPtr->IDNFragmentedFlag = false;								// Reset fragment flag
		return;															// Success

	case IDNCMD_SCAN_REQUEST:											// Network scan request?

		DataPtr->IDNResponseData->PacketCount = PacketCount;			// Send response using the received packet sequence count
		IDNSendScanResponse(DataPtr->IDNResponseData);					// Send scan response
		return;															// Success

	case IDNCMD_SERVICEMAP_REQUEST:

		DataPtr->IDNResponseData->PacketCount = PacketCount;			// Send response using the received packet sequence count
		IDNSendServiceMapResponse(DataPtr->IDNResponseData);			// Send service map response
		return;															// Success

	// Handle more packet commands - ToDo

	default:
		if (ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Unknown/unsupported packet command (cmd=0x%x).\n", ++ErrorCount, PacketCommand);
		goto Error;
	}

	/* Check packet count sequence */

	if (DataPtr->IDNSyncPacketCountFlag == true || PacketCount < NextPacketCount){	// Resync packet count?
		
		if (DataPtr->IDNSyncPacketCountFlag == false && ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Warning, received packet count is less than expected (overflow?) - resyncing packet count (%d<%d)...\n", ErrorCount, PacketCount, NextPacketCount);
		
		DataPtr->IDNSyncPacketCountFlag = false;						// Just once

		NextPacketCount = PacketCount;									// Resync packet sequence count

		for (i = 0; i < IDNMAXRESYNC; i++)								// Restart resync buffer
			ResyncSize[i] = 0;											// Not valid

		TrueLastTimestamp_us = 0;										// No true last timestamp yet

	} else {

		if (PacketCount != NextPacketCount){							// Check packet count

			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: Warning, received out of order packet (%d!=%d)...\n", ErrorCount, PacketCount, NextPacketCount);

			for (i = 0; i < IDNMAXRESYNC; i++)							// Find next available resync buffer
				if (ResyncSize[i] == 0)									// Not valid?
					break;

			if (i == IDNMAXQUEUE){										// Just in case
				if (ExtendedErrorMsgsFlag == true)
					printf("(%d) IDN: Resync buffer is corrupted (should never get here)!\n", ++ErrorCount);
				goto Error;
			}

			memcpy((DataPtr->IDNMemPtr + IDNRESYNCBUFFOFFSET + (i * IDNMAXPACKETSIZE)), (DataPtr->IDNMemPtr + IDNINPUTBUFFOFFSET), Size);	// Store out of order packet
			ResyncSize[i] = Size;										// Valid

			for (i = 0; i < IDNMAXRESYNC; i++)							// Check if resync buffer full
				if (ResyncSize[i] == 0)									// Not valid?
					break;

			if (i == IDNMAXRESYNC){										// Buffer full?									
				if (ExtendedErrorMsgsFlag == true)
					printf("(%d) IDN: Detected lost packet - skipping to next valid packet (lostcount=%d)...\n", ++ErrorCount, ++LostPacketCount);

				PacketCount = 0xffff;									// Start on maximum count
				
				for (i = 0; i < IDNMAXRESYNC; i++){						// Find lowest valid packet count
					PacketHeaderTempPtr = (IDNHDR_PACKET *)(DataPtr->IDNMemPtr + IDNRESYNCBUFFOFFSET + (i * IDNMAXPACKETSIZE));
					PacketCountTemp = ntohs(PacketHeaderTempPtr->sequence);	// Get PacketCount
					if (PacketCountTemp <= PacketCount){					// Lowest packet?
						PacketHeaderPtr = PacketHeaderTempPtr;				// Use this packet
						PacketCount = PacketCountTemp;
						Temp = i;
					}
				}

				NextPacketCount = PacketCount;							// Resync packet sequence count

				if (ExtendedErrorMsgsFlag == true)
					printf("(%d) IDN: Resynced to next valid packet (count=%d).\n", ErrorCount, PacketCount);
				PacketCommand = PacketHeaderPtr->command;				// Get PacketCommand (single byte)
//				PacketFlags = PacketHeaderPtr->flags;					// Get PacketFlags (single byte)
				Size = ResyncSize[Temp];								// Get data size
				ResyncSize[Temp] = 0;									// Used - not valid
				goto Next;
			}
			
			for (i = 0; i < IDNMAXRESYNC; i++)							// Try to find expected packet
				if (ResyncSize[i] != 0){								// Valid?
					PacketHeaderPtr = (IDNHDR_PACKET *)(DataPtr->IDNMemPtr + IDNRESYNCBUFFOFFSET + (i * IDNMAXPACKETSIZE));
					PacketCount = ntohs(PacketHeaderPtr->sequence);		// Get PacketCount
					if (PacketCount == NextPacketCount)					// Found correct packet?
						break;
				}

			if (i == IDNMAXRESYNC){
				if (ExtendedErrorMsgsFlag == true)
					printf("(%d) IDN: Could not resync out of order packet - waiting for more packets...\n", ErrorCount);
				return;													// Wait for next packet
			}

			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: Successfully resynced out of order packet (count=%d).\n", ErrorCount, PacketCount);
			PacketCommand = PacketHeaderPtr->command;					// Get PacketCommand (single byte)
//			PacketFlags = PacketHeaderPtr->flags;						// Get PacketFlags (single byte)
			Size = ResyncSize[i];										// Get data size
			ResyncSize[i] = 0;											// Used - not valid
		}
	}

Next:

	NextPacketCount++;													// Update expected packet sequence count
	
	if (Size == sizeof(IDNHDR_PACKET)){									// No message?
		if (ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Received packet has no message.\n", ++ErrorCount);
		goto Error;
	}

	if (Size < (sizeof(IDNHDR_PACKET) + sizeof(IDNHDR_CHANNEL_MESSAGE))){	// Check data size
		if (ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Received message data is too small (size=%d)!\n", ++ErrorCount, Size);
		goto Error;
	}

	ChannelMsgHeaderPtr = (IDNHDR_CHANNEL_MESSAGE *)&PacketHeaderPtr[1];	// Point to message header
	TotalSize = ntohs(ChannelMsgHeaderPtr->totalSize);						// Get TotalSize
	ContentID = ntohs(ChannelMsgHeaderPtr->contentID);						// Get ContentID (ID + flags)
	Timestamp_us = ntohl(ChannelMsgHeaderPtr->timestamp);					// Get Timestamp (microseconds - overflows in 71 minutes!)
	ChannelID = ((ContentID & IDNMSK_CONTENTID_CHANNELID) >> 8);			// Extract channel ID
	ChunkType = (ContentID & IDNMSK_CONTENTID_CNKTYPE);						// Extract chunk type

	if (ChunkType != IDNVAL_CNKTYPE_VOID && DataPtr->IDNFragmentedFlag == false){	// Ignore timestamps of "keep alive" packets and fragments

		if (TrueLastTimestamp_us != 0){									// Resync timestamp?
			if (LastTimestamp_us > Timestamp_us){
				if (ExtendedErrorMsgsFlag == true)
					printf("(%d) IDN: Warning, message timestamp is before last message - timer overflow (timestamp=%uus)?\n", ++ErrorCount, Timestamp_us);
				TrueLastTimestamp_us += (0xffffffff - LastTimestamp_us + Timestamp_us + 1);	// Update true last timestamp relative to last message timestamp (converts to 64 bit)
			} else {
				if (LastTimestamp_us == Timestamp_us){
					if (ExtendedErrorMsgsFlag == true)
						printf("(%d) IDN: Received message timestamp is the same as the last message (timestamp=%uus)!\n", ++ErrorCount, Timestamp_us);
					goto Error;
				}
				TrueLastTimestamp_us += (Timestamp_us - LastTimestamp_us);	// Update true last timestamp relative to last message timestamp (converts to 64 bit)
			}
		} else {
			TrueLastTimestamp_us = Timestamp_us;						// Set first true last timestamp
		}

		LastTimestamp_us = Timestamp_us;								// Update last timestamp
	}
	
	if (TotalSize != (Size - sizeof(IDNHDR_PACKET))){					// Check data size
		if (ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Message total size does not match received message size (%d!=%d)!\n", ++ErrorCount, TotalSize, (int)(Size - sizeof(IDNHDR_PACKET)));
		goto Error;
	}

	if ((ContentID & IDNFLG_CONTENTID_CHANNELMSG) == 0){				// Check valid channel message
		if (ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Content ID message flag not set (0x%x&0x%x).\n", ++ErrorCount, ContentID, IDNFLG_CONTENTID_CHANNELMSG);
		goto Error;
	}

	if (ChannelID > IDNMAXCHANNELID){									// Check Channel ID is within my range
		if (ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Channel ID out of supported range (%d>%d).\n", ++ErrorCount, ChannelID, IDNMAXCHANNELID);
		goto Error;
	}
			
	if (DataPtr->IDNFragmentedFlag == false){							// Expecting fragments?

		if (ContentID & IDNFLG_CONTENTID_CONFIG_LSTFRG){				// Config header follows?

			if (Size < (sizeof(IDNHDR_PACKET) + sizeof(IDNHDR_CHANNEL_MESSAGE) + sizeof(IDNHDR_CHANNEL_CONFIG))){	// Check data size
				if (ExtendedErrorMsgsFlag == true)
					printf("(%d) IDN: Received message data is too small (size=%d)!\n", ++ErrorCount, Size);
				goto Error;
			}

			ChannelConfigHeaderPtr = (IDNHDR_CHANNEL_CONFIG *)&ChannelMsgHeaderPtr[1];
			CfgWordCount = ChannelConfigHeaderPtr->wordCount;			// Get config wordcount (single byte)
			CfgFlags = ChannelConfigHeaderPtr->flags;					// Get config flags (single byte)
//			CfgServiceID = ChannelConfigHeaderPtr->serviceID;			// Get config serviceID (single byte)
			CfgServiceMode = ChannelConfigHeaderPtr->serviceMode;		// Get config serviceMode (single byte)
							
			if (CfgFlags & IDNFLG_CHNCFG_CLOSE){						// Close channel?
				if (ExtendedErrorMsgsFlag == true){
					if (DataPtr->IDNCurrentChannelID != 0xff)
						printf("(%d) IDN: Received channel close flag - channel %d is now closed.\n", ErrorCount, ChannelID);
					else
						printf("(%d) IDN: Received channel close flag - channel %d is already closed.\n", ErrorCount, ChannelID);
				}
				DataPtr->IDNCurrentChannelID = 0xff;					// Channel closed (if not already)
				DataPtr->IDNFragmentedFlag = false;						// Reset fragment flag
				goto Done;												// Success
			}

			if (CfgFlags & IDNFLG_CHNCFG_ROUTING){						// Configure and open channel?

				switch (CfgServiceMode){

				case IDNVAL_SMOD_VOID:									// No function, no lookup
					DataPtr->IDNFragmentedFlag = false;					// Reset fragment flag
					goto Done;											// Ignore

				case IDNVAL_SMOD_LPGRF_DISCRETE:						// Laser graphic: Stream of individual frames
					DataPtr->IDNWaveformMode = false;					// Set mode to frames
					break;												// Supported

				case IDNVAL_SMOD_LPGRF_CONTINUOUS:						// Laser graphic: Stream of waveform segments
					DataPtr->IDNWaveformMode = true;					// Set mode to waveform
					break;												// Supported
				
				default:
					if (ExtendedErrorMsgsFlag == true)
						printf("(%d) IDN: Unknown/unsupported service mode (mode=0x%x).\n", ++ErrorCount, CfgServiceMode);
					goto Error;
				}

				/* Verify, open and config channel */

				if (Size < ((CfgWordCount * 4) + sizeof(IDNHDR_PACKET) + sizeof(IDNHDR_CHANNEL_MESSAGE) + sizeof(IDNHDR_CHANNEL_CONFIG))){	// Check data size
					if (ExtendedErrorMsgsFlag == true)
						printf("(%d) IDN: Received channel configuration data is too small (size=%d)!\n", ++ErrorCount, Size);
					goto Error;
				}
								
				DescriptorsPtr = (unsigned short *)&ChannelConfigHeaderPtr[1];	// Point to descriptors

				Temp = 0;												// Default to unknown

				if (CfgWordCount == 4){
					if (ntohs(DescriptorsPtr[0]) == 0x4200 &&			// X
						ntohs(DescriptorsPtr[1]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[2]) == 0x4210 &&			// Y
						ntohs(DescriptorsPtr[3]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[4]) == 0x527E &&			// Red (638nm) - 8 bits
						ntohs(DescriptorsPtr[5]) == 0x5214 &&			// Green (532nm) - 8 bits
						ntohs(DescriptorsPtr[6]) == 0x51CC &&			// Blue (460nm) - 8 bits
						ntohs(DescriptorsPtr[7]) == 0x0000){			// Alignment
							Temp = 7;									// Format = 7 bytes
							TempPtr = (char *)"XYrgb";
					}
					
					if (ntohs(DescriptorsPtr[0]) == 0x4200 &&			// X
						ntohs(DescriptorsPtr[1]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[2]) == 0x4210 &&			// Y
						ntohs(DescriptorsPtr[3]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[4]) == 0x527E &&			// Red (638nm) - 8 bits
						ntohs(DescriptorsPtr[5]) == 0x5214 &&			// Green (532nm) - 8 bits
						ntohs(DescriptorsPtr[6]) == 0x51CC &&			// Blue (460nm) - 8 bits
						ntohs(DescriptorsPtr[7]) == 0x5C10){			// Intensity - 8 bits
							Temp = 8;									// Format = 8 bytes
							TempPtr = (char *)"XYrgbi";
					}

				}

				if (CfgWordCount == 5){
					if (ntohs(DescriptorsPtr[0]) == 0x4200 &&			// X
						ntohs(DescriptorsPtr[1]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[2]) == 0x4210 &&			// Y
						ntohs(DescriptorsPtr[3]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[4]) == 0x527E &&			// Red (638nm)
						ntohs(DescriptorsPtr[5]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[6]) == 0x5214 &&			// Green (532nm)
						ntohs(DescriptorsPtr[7]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[8]) == 0x51CC &&			// Blue (460nm)
						ntohs(DescriptorsPtr[9]) == 0x4010){			// 16 Bit
							Temp = 10;									// Format = 10 bytes
							TempPtr = (char *)"XYRGB";
					}
				}

				if (CfgWordCount == 10){
					if (ntohs(DescriptorsPtr[0]) == 0x4200 &&			// X
						ntohs(DescriptorsPtr[1]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[2]) == 0x4210 &&			// Y
						ntohs(DescriptorsPtr[3]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[4]) == 0x527E &&			// Red (638nm)
						ntohs(DescriptorsPtr[5]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[6]) == 0x5214 &&			// Green (532nm)
						ntohs(DescriptorsPtr[7]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[8]) == 0x51CC &&			// Blue (460nm)
						ntohs(DescriptorsPtr[9]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[10]) == 0x5C10 &&			// Intensity
						ntohs(DescriptorsPtr[11]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[12]) == 0x51BD &&			// User 1 (deep blue)
						ntohs(DescriptorsPtr[13]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[14]) == 0x5241 &&			// User 2 (yellow)
						ntohs(DescriptorsPtr[15]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[16]) == 0x51E8 &&			// User 3 (cyan)
						ntohs(DescriptorsPtr[17]) == 0x4010 &&			// 16 Bit
						ntohs(DescriptorsPtr[18]) == 0x4201 &&			// User 4 (X-prime)
						ntohs(DescriptorsPtr[19]) == 0x4010){			// 16 Bit
							Temp = 20;									// Format = 20 bytes
							TempPtr = (char *)"XYRGBI1234";
					}
				}

				if (Temp == 0){											// Unknown format?
					if (ExtendedErrorMsgsFlag == true)
						printf("(%d) IDN: Unknown/unsupported channel configuration format (wordcount=%d).\n", ++ErrorCount, CfgWordCount);
					goto Error;
				}
								
				if (DataPtr->IDNCurrentChannelID == 0xff){				// Channel open?
					if (ExtendedErrorMsgsFlag == true)
						printf("(%d) IDN: New channel configuration received - channel %d is now open and configured to \"%s\" (%d bytes).\n", ErrorCount, ChannelID, TempPtr, Temp);
				} else {
					if (ChannelID != DataPtr->IDNCurrentChannelID){		// Channel ID missmatch?
						if (ExtendedErrorMsgsFlag == true)
							printf("(%d) IDN: Unexpected channel ID - only single channel is supported (%d!=%d).\n", ++ErrorCount, ChannelID, DataPtr->IDNCurrentChannelID);
						goto Error;
					}
					if (Temp != DataPtr->IDNFormat){					// Configuration missmatch (verify)?
						if (ExtendedErrorMsgsFlag == true)
							printf("(%d) IDN: Channel configuration verify missmatch (%d!=%d)! (\n", ++ErrorCount, Temp,  DataPtr->IDNFormat);
						goto Error;
					}
				}
				
				DataPtr->IDNFormat = Temp;								// Set format
				DataPtr->IDNCurrentChannelID = ChannelID;				// Open this channel (single channel only)									
				SampleChunkHeaderPtr = (IDNHDR_SAMPLE_CHUNK *)&DescriptorsPtr[(CfgWordCount * 2)];	// Point to sample chunk header
				
			} else {

				if (ExtendedErrorMsgsFlag == true)
					printf("(%d) IDN: Unknown/unsupported channel config flag (flags=0x%x).\n", ++ErrorCount, CfgFlags);
				goto Error;
			}
			
		} else {

			SampleChunkHeaderPtr = (IDNHDR_SAMPLE_CHUNK *)&ChannelMsgHeaderPtr[1];	// Point to sample chunk header
		}

		if (TotalSize == sizeof(IDNHDR_CHANNEL_MESSAGE)){				// Any sample chunk?
			DataPtr->IDNFragmentedFlag = false;							// Reset fragment flag
			goto Done;													// Ignore
		}
	
		DataChunkStartPtr = (unsigned char *)&SampleChunkHeaderPtr[1];	// Point to start of new chunk data
		Temp = (unsigned int)((DataChunkStartPtr - (unsigned char *)PacketHeaderPtr) + (DataPtr->IDNFormat * 2));	// Minimum of 2 points total needed

		if (Size < Temp){												// Check data size
			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: New sample chunk data too small - less than 2 points (%d<%d)!\n", ++ErrorCount, Size, Temp);
			goto Error;
		}

		Period = (ntohl(SampleChunkHeaderPtr->flagsDuration) & 0x00ffffff);	// Extract Period (must be after size check)

	} else {

		DataChunkStartPtr = (unsigned char *)&ChannelMsgHeaderPtr[1];	// Point to start of fragmented chunk data

		Temp = (unsigned int)((DataChunkStartPtr - (unsigned char *)PacketHeaderPtr) + 1);	// Minimum of 1 byte needed

		if (Size < Temp){												// Check data size
			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: No sample chunk data received (%d<%d)!\n", ++ErrorCount, Size, Temp);
			goto Error;
		}
	}

	if (ChannelID != DataPtr->IDNCurrentChannelID){						// Check channel is open and configured
		if (ExtendedErrorMsgsFlag == true){
			if (DataPtr->IDNCurrentChannelID == 0xff)
				printf("(%d) IDN: Requested channel ID is not open and configured yet (ID=%d).\n", ++ErrorCount, ChannelID);
			else
				printf("(%d) IDN: Unexpected channel ID - only single channel is supported (%d!=%d).\n", ++ErrorCount, ChannelID, DataPtr->IDNCurrentChannelID);
		}
		goto Error;
	}

	switch (ChunkType){
	
	case IDNVAL_CNKTYPE_VOID:											// Empty data chunk (keep alive)
		DataPtr->IDNFragmentedFlag = false;								// Reset fragment flag
		goto Done;														// Ignore

	case IDNVAL_CNKTYPE_LPGRF_FRAME:									// Complete frame

		if (DataPtr->IDNFragmentedFlag == true || DataPtr->IDNWaveformMode == true){	// Just in case
			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: Unexpected frame complete data chunk!\n", ++ErrorCount);
			goto Error;
		}

		break;															// Fall through to std frame process

	case IDNVAL_CNKTYPE_LPGRF_FRAME_FIRST:								// Fragmented frame (first)
		
		if (DataPtr->IDNFragmentedFlag == true || DataPtr->IDNWaveformMode == true){	// Just in case
			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: Unexpected frame first data chunk!\n", ++ErrorCount);
			goto Error;
		}
				
		Temp = (Size - (unsigned int)(DataChunkStartPtr - (unsigned char *)PacketHeaderPtr));	// Calc chunk data size

		if (Temp > IDNMAXSAMPLESIZE){									// Just in case
			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: Received fragmented chunk data will exceed buffer size (%d>%d)!\n", ++ErrorCount, Temp, IDNMAXSAMPLESIZE);
			goto Error;
		}

		memcpy(DataPtr->IDNMemPtr, DataChunkStartPtr, Temp);			// Assemble fragment(s) in frag buffer
		DataEOFPtr = (DataPtr->IDNMemPtr + Temp);						// Point to next (End Of File)
		DataPtr->IDNFragmentedFlag = true;								// Enable fragmented mode
		goto Done;														// Wait for more data

	case IDNVAL_CNKTYPE_LPGRF_FRAME_SEQUEL:								// Fragmented frame (sequel)

		if (DataPtr->IDNFragmentedFlag == false || DataPtr->IDNWaveformMode == true){	// Just in case
			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: Unexpected frame sequel data chunk!\n", ++ErrorCount);
			goto Error;
		}
	
		Temp = (Size - (unsigned int)(DataChunkStartPtr - (unsigned char *)PacketHeaderPtr));	// Calc size

		if ((DataEOFPtr - DataPtr->IDNMemPtr + Temp) > IDNMAXSAMPLESIZE){	// Just in case
			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: Received fragmented chunk data will exceed buffer size (%d>%d)!\n", ++ErrorCount, (int)(DataEOFPtr - DataPtr->IDNMemPtr + Temp), IDNMAXSAMPLESIZE);
			goto Error;
		}
		
		memcpy(DataEOFPtr, DataChunkStartPtr, Temp);					// Append this fragment
		DataEOFPtr += Temp;												// Point to next (End Of File)

		if (ContentID & IDNFLG_CONTENTID_CONFIG_LSTFRG)					// Final?
			break;														// Fall through to std frame process
	
		goto Done;														// Wait for more data


	case IDNVAL_CNKTYPE_LPGRF_WAVE:										// Waveform

		if (DataPtr->IDNFragmentedFlag == true || DataPtr->IDNWaveformMode == false){	// Just in case
			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: Unexpected waveform data chunk!\n", ++ErrorCount);
			goto Error;
		}

		/* Append waveform chunk data to newest queued frame to prevent Q from overflowing too quickly */
		
		if (DataPtr->UpdateScope == false)								// Buffer ready?
			break;														// Fall through to std frame process (send now)
		
		if (DataPtr->IDNQueueCount == 0)								// Any queued frames?
			break;														// Fall through to std frame process (will start queue)
		
		if (DataPtr->IDNQueueCount == 1 && DataPtr->IDNUnqueueFlag == true)	// Last frame already sent?
			break;														// Fall through to std frame process (will add new queued frame)
				
		for (i = 0; i < IDNMAXQUEUE; i++){								// Find most recent queued frame
			if (DataPtr->IDNQueueData[i].Sequence == DataPtr->IDNQueueCount)
				break;
		}

		if (i == IDNMAXQUEUE)											// Just in case
			break;														// Fall through to std frame process (to handle corrupted queue)													 

		if (DataPtr->IDNQueueData[i].Points >= IDNMAXWAVEFORMPOINTS)	// Got minimum points already?
			break;														// Fall through to std frame process (have enough points already)
		
		Temp = (Size - (unsigned int)(DataChunkStartPtr - (unsigned char *)PacketHeaderPtr));	// Calc size

		if (Temp % DataPtr->IDNFormat){									// Must divide exactly
			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: Unexpected waveform sample chunk data size - possible format missmatch!\n", ++ErrorCount);
			goto Error;
		}

		Points = (Temp / DataPtr->IDNFormat);							// Calc points

		if ((DataPtr->IDNQueueData[i].Points + Points) > IDNMAXPOINTS)	// Just in case
			break;														// Fall through to std frame process (too many total points)

		PPS = (((unsigned long long)Points * (unsigned long long)1000000) / (unsigned long long)Period);	// Calc PPS

		if (PPS < MINPPS)												// At least 1
			PPS = MINPPS;
		else
			if (PPS > IDNMAXDACPPS)										// Check PPS
				PPS = IDNMAXDACPPS;										// Cap PPS to hardware limit

		if (PPS >= DataPtr->IDNQueueData[i].PPS)						// Calc PPS difference
			Temp = PPS - DataPtr->IDNQueueData[i].PPS;
		else
			Temp = DataPtr->IDNQueueData[i].PPS - PPS;

		if (Temp > 2){													// PPS difference should be close (i.e. ignore round off errors)
//			printf("(%d) IDN: Warning, unable to appended waveform as PPS differs to queued frame by too much (%d>2)...\n", ErrorCount, Temp);	// Testing only
			break;														// Fall through to std frame process (as PPS does not match)
		}
				
		DataPtr->IDNQueueData[i].Checksum = MakeIDNDACData(DataChunkStartPtr, (unsigned short *)(DataPtr->IDNMemPtr + IDNMAXSAMPLESIZE + (i * IDNQOUTBUFFSIZE)), Points, DataPtr->IDNFormat, DataPtr->IDNQueueData[i].Points, DataPtr->IDNQueueData[i].Checksum);
//		printf("Appended (%d): %d + %d = %d, checksum=%d (i=%d)\n", ++DataPtr->IDNQueueData[i].AppendCount, DataPtr->IDNQueueData[i].Points, Points, (DataPtr->IDNQueueData[i].Points + Points), DataPtr->IDNQueueData[i].Checksum, i); // Debug/testing only
		DataPtr->IDNQueueData[i].Points += Points;						// Update with new total points
		DataPtr->IDNQueueData[i].Period += Period;						// Update with new total period					
		goto Done;														// Wait for more data
		
	default:
		
		if (ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Unknown/unsupported chunk type (type=0x%x).\n", ++ErrorCount, ChunkType);
		goto Error;
	}

	/* Make DAC data */

	if (DataPtr->IDNFragmentedFlag == false)							// Calc sample size
		Temp = (Size - (unsigned int)(DataChunkStartPtr - (unsigned char *)PacketHeaderPtr));
	else
		Temp = (unsigned int)(DataEOFPtr - DataPtr->IDNMemPtr);

	if (Temp % DataPtr->IDNFormat){										// Must divide exactly
		if (ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Unexpected sample chunk data size - possible format missmatch!\n", ++ErrorCount);
		goto Error;
	}

	Points = (Temp / DataPtr->IDNFormat);								// Calc points

	if (Points > IDNMAXPOINTS){											// Just in case
		if (ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Too many points received (points=%d).\n", ++ErrorCount, Points);
		goto Error;
	}

	PPS = (((unsigned long long)Points * (unsigned long long)1000000) / (unsigned long long)Period);	// Calc PPS

	if (PPS < MINPPS){													// At least 1
		if (ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Warning, frame PPS was below minimum limit - PPS set to minimum instead... (%d<%d).\n", ++ErrorCount, PPS, MINPPS);
		PPS = MINPPS;
	} else {
		if (PPS > IDNMAXDACPPS){										// Check PPS
			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: Warning, frame PPS exceeds LASERfIREDAC hardware limit - expect frame queue overwrites (frame drops) (%d>%d)...\n", ErrorCount, PPS, IDNMAXDACPPS);
			PPS = IDNMAXDACPPS;											// Cap PPS to hardware limit
		}
	}

	if (DataPtr->UpdateScope == true){									// Buffer ready?

		for (i = 0; i < IDNMAXQUEUE; i++){								// Find next free queue slot
			if (DataPtr->IDNQueueData[i].Sequence == 0)
				break;
		}

		if (i == IDNMAXQUEUE){
			
			if (ExtendedErrorMsgsFlag == true)
				printf("(%d) IDN: Frame queue is full - overwriting most recent queued frame...\n", ErrorCount);
			
			for (i = 0; i < IDNMAXQUEUE; i++){							// Find most recent frame
				if (DataPtr->IDNQueueData[i].Sequence == DataPtr->IDNQueueCount)
					break;
			}

			if (i == IDNMAXQUEUE){										// Just in case
				if (ExtendedErrorMsgsFlag == true)
					printf("(%d) IDN: Queued IDN frame data is corrupted (should never get here)!\n", ++ErrorCount);
				goto Error;
			}
				
		} else {
			DataPtr->IDNQueueCount++;									// Increase queue count
		}
				
		DataPtr->IDNQueueData[i].PPS = PPS;
		DataPtr->IDNQueueData[i].Points = Points;											
		DataPtr->IDNQueueData[i].Period = Period;											
		DataPtr->IDNQueueData[i].Checksum = MakeIDNDACData((DataPtr->IDNFragmentedFlag == false ? DataChunkStartPtr : DataPtr->IDNMemPtr), (unsigned short *)(DataPtr->IDNMemPtr + IDNMAXSAMPLESIZE + (i * IDNQOUTBUFFSIZE)), Points, DataPtr->IDNFormat, 0, 0);
		DataPtr->IDNQueueData[i].Sequence = DataPtr->IDNQueueCount;
		DataPtr->IDNFragmentedFlag = false;								// Got all fragments
//		DataPtr->IDNQueueData[i].AppendCount = 0;						// Debug/testing only

		Temp = 0;														// Calc total queued frame periods
		for (i = 0; i < IDNMAXQUEUE; i++)
			if (DataPtr->IDNQueueData[i].Sequence != 0)
				Temp += DataPtr->IDNQueueData[i].Period;

		Temp = (Temp / DataPtr->IDNQueueCount);							// Calc average period
		
		if (Temp == 0)													// Just in case (division by zero!)
			Temp = 1;

		Temp = (IDNQUEUEDTARGETPERIOD / Temp);							// Get queue count target from average period 
		
		if (Temp < IDNMINIQUEUEDTARGET)									// Make sure target is within range
			Temp = IDNMINIQUEUEDTARGET;
		else
			if (Temp > IDNMAXIQUEUEDTARGET)
				Temp = IDNMAXIQUEUEDTARGET;

		DataPtr->IDNQueueTarget = Temp;									// Update queue traget
		DataPtr->IDNSpeedAdjust = 100;									// Set default speed adjust

		if (DataPtr->IDNQueueCount < Temp){								// Decrease speed adjust if lower than target (caps at 1 -> 100 - ((6 - 1) * 2.5) = 88)
			if ((DataPtr->IDNSpeedAdjust = (100 - (((Temp - DataPtr->IDNQueueCount) * 5) / 2))) < 88)
				DataPtr->IDNSpeedAdjust = 88;
		} else {
			if (DataPtr->IDNQueueCount > Temp)							// Increase speed adjust if higher than target (caps at 40 -> 100 + ((40 - 6) * 3) = 202)		
				if ((DataPtr->IDNSpeedAdjust = (100 + ((DataPtr->IDNQueueCount - Temp) * 3))) > 200)
					DataPtr->IDNSpeedAdjust = 200;
		}

		SpeedAdjustRestoreCount = IDNMAXIQUEUEDTARGET;					// Give up on regulating speed after this many non-queued frames

	} else {

		if (DataPtr->IDNWaveformMode == true && DataPtr->IDNSpeedAdjust >= 100){	// Try to restart Q for waveform mode
			DataPtr->IDNSpeedAdjust = 97;								// Changed 98 -> 97
			SpeedAdjustRestoreCount = IDNMAXIQUEUEDTARGET;				// Give up on regulating speed after this many non-queued frames
		}

		if ((PPS = ((DataPtr->IDNSpeedAdjust * PPS) / 100)) > IDNMAXDACPPS)	// Calc adjusted PPS 
			PPS = IDNMAXDACPPS;												// Cap PPS to hardware limit
		else
			if (PPS < MINPPS)
				PPS = MINPPS;
		
		DataPtr->PPS = PPS;
		DataPtr->TotalPoints = Points;
		DataPtr->IDNBuffPtr = (DataPtr->IDNMemPtr + IDNOUTPUTBUFFOFFSET);	// Point to output buffer
		DataPtr->IDNChecksum = MakeIDNDACData((DataPtr->IDNFragmentedFlag == false ? DataChunkStartPtr : DataPtr->IDNMemPtr), (unsigned short *)DataPtr->IDNBuffPtr, Points, DataPtr->IDNFormat, 0, 0);
		DataPtr->IDNFragmentedFlag = false;								// Got all fragments
		DataPtr->UpdateScope = true;									// Signal new buffer ready
		DataPtr->Freq = CalcFreq(PPS, Points);							// Update Freq
								
		if (SpeedAdjustRestoreCount != 0){								// Restore speed adjust?
			SpeedAdjustRestoreCount--;
		} else {
			if (DataPtr->IDNSpeedAdjust < 100)							// Slowly set speed adjust back to 100%
				DataPtr->IDNSpeedAdjust++;
			else
				if (DataPtr->IDNSpeedAdjust > 100)
					DataPtr->IDNSpeedAdjust--;
			if (DataPtr->IDNSpeedAdjust == 100)
				DataPtr->IDNQueueTarget = 0;							// Reset queue traget
		}
	}

Done:

	for (i = 0; i < IDNMAXRESYNC; i++)									// Look for next packet from resync buffer
		if (ResyncSize[i] != 0){										// Valid?
			PacketHeaderPtr = (IDNHDR_PACKET *)(DataPtr->IDNMemPtr + IDNRESYNCBUFFOFFSET + (i * IDNMAXPACKETSIZE));
			PacketCount = ntohs(PacketHeaderPtr->sequence);				// Get PacketCount
			if (PacketCount == NextPacketCount)							// Found next packet?
				break;
		}

	if (i != IDNMAXRESYNC){												// Already got next packet?
		if (ExtendedErrorMsgsFlag == true)
			printf("(%d) IDN: Successfully resynced out of order packet (count=%d).\n", ErrorCount, PacketCount);
		PacketCommand = PacketHeaderPtr->command;						// Get PacketCommand (single byte)
//		PacketFlags = PacketHeaderPtr->flags;							// Get PacketFlags (single byte)
		Size = ResyncSize[i];											// Get data size
		ResyncSize[i] = 0;												// Used - not valid
		goto Next;
	}

	return;																// Complete

Error:
	DataPtr->IDNSyncPacketCountFlag = true;								// Resync packet count - just in case
	DataPtr->IDNFragmentedFlag = false;									// Reset fragment flag
	DataPtr->LastPPS = -1;												// Force status update because status may have scrolled because of printf()

	// Add extra error handling here

	return;	
}


bool IDNDACUpdate(LASERFIREDATA *DataPtr)
{
	static MYTIMERRETURN CurrentTimestamp = MYTIMER();
	static unsigned int NewBufferPeriod_us = 0, CurrentDuration_us = 20000;
	MYTIMERRETURN CurrentTime;
	unsigned int Points, PPS, DataSize, Temp;
	
	/* Double buffered DAC timing emulator */
	
	if (NewBufferPeriod_us != 0){										// Got a buffer update waiting?
		CurrentTime = MYTIMER();										// Get current time
		Temp = (unsigned int)(CurrentTime - CurrentTimestamp);
		if (Temp >= CurrentDuration_us){
			Temp -= CurrentDuration_us;									// Calc new buffer simulated run time
			CurrentDuration_us = NewBufferPeriod_us;
			CurrentTimestamp = (CurrentTime - (Temp % CurrentDuration_us));	// Calc current buffer simulated start timestamp
			NewBufferPeriod_us = 0;											// Buffer ready
		}
	}

	if (NewBufferPeriod_us != 0)										// Buffer ready?
		return false;

	if (DataPtr == 0)													// Just checking if buffer ready only?
		return true;

	
	// Replace emulator code above with real DAC hardware buffer ready check //


	if (DataPtr->IDNBuffPtr == 0)										// Just in case (IDN mode has no starting circle)
		return true;

	Points = DataPtr->TotalPoints;
	PPS = DataPtr->PPS;
	DataPtr->FramePeriod_us = NewBufferPeriod_us = (((unsigned long long)Points * (unsigned long long)1000000) / (unsigned long long)PPS);	// Used by emulator and main loop to limit sleep time
	DataSize = (((Points * 15) + 1) / 2);								// Points x 7.5 rounded up


	// Write to DAC here with the "DataPtr->IDNBuffPtr" pointer, "DataSize" (or "Points") and "PPS" //
	// DataPtr->IDNChecksum can also be used by the DAC hardware for data validation                //
	// Data format: [YX * points][rgb * points][TTL_nibbles * points] (7.5 bytes / point)           //


	DataPtr->FrameTimestamp = MYTIMER();								// Used by main loop to limit sleep time

	if (DataPtr->UpdateScope == true && SIMWindow != 0){				// Buffer changed and SIM window active?
		if (DataPtr->SIMBuffSwitch == false){							// SIM() controls which buffer to use
			memcpy(DataPtr->SIMMemPtr, DataPtr->IDNBuffPtr, DataSize);	// Copy DAC data to SIM buffer
			DataPtr->SIMBuffPtr = DataPtr->SIMMemPtr;					// Tell SIM() which buffer to use
			DataPtr->SIMPoints = Points;								// Tell SIM() points
		} else {
			memcpy((DataPtr->SIMMemPtr + SIMBUFFSIZE), DataPtr->IDNBuffPtr, DataSize);	// Copy DAC data to SIM buffer 
			DataPtr->SIMBuffPtr = (DataPtr->SIMMemPtr + SIMBUFFSIZE);					// Tell SIM() which buffer to use
			DataPtr->SIMPoints = Points;												// Tell SIM() points
		}
	}

	return true;
}


void DrawScope(HWND WinHandle, HDC DCHandle, LASERFIREDATA *DataPtr, unsigned char *SIMBuffPtr, unsigned int SIMPoints)	// Asynchronous function call
{
	static HBRUSH Brush = 0;
	unsigned short *DACDataPtr;
	unsigned char *DACTTLDataPtr, *DACColourDataPtr = 0;
	POINT *Points;
	unsigned int FirstColour, Colour, LastColour, i, a;
	unsigned int FirstX, X, FirstY, Y;
	unsigned char TTL, R, G, B;
	bool ShutterFlag;
	HPEN Pen;
	HGDIOBJ OrgObject;
	RECT Rect;

	if (Brush == 0)														// Initalize Brush
		Brush = CreateSolidBrush(0);

	Rect.left = 3;														// Draw 3D edge
	Rect.top = 3;
	Rect.right = (SCOPESIZE + 10);
	Rect.bottom = (SCOPESIZE + 10);
	DrawEdge(DCHandle, &Rect, EDGE_SUNKEN, BF_RECT);

	Rect.left = 5;														// Erase current scope
	Rect.top = 5;
	Rect.right = (SCOPESIZE + 8);
	Rect.bottom = (SCOPESIZE + 8);
	FillRect(DCHandle, &Rect, Brush);

	if (DataPtr == 0 || SIMBuffPtr == 0 || SIMPoints == 0)				// Just in case
		return;

	DACDataPtr = (unsigned short *)SIMBuffPtr;							// Point to DAC data
	DACColourDataPtr = ((unsigned char *)SIMBuffPtr + (SIMPoints * 4));	// Point to colour data
	DACTTLDataPtr = ((unsigned char *)SIMBuffPtr + (SIMPoints * 7));	// Point to TTL data

	ShutterFlag = DataPtr->ShutterFlag;

	if (DrawBetweenPointsFlag == false){

		for (i = 0; i < SIMPoints; i++){								// Draw points only

			Y = *DACDataPtr++;											// Get Y
			X = *DACDataPtr++;											// Get X

			if (i & 1)
				TTL = (*DACTTLDataPtr++ >> 4);							// Get TTL nibble
			else
				TTL = *DACTTLDataPtr;									// Get TTL nibble

			R = *DACColourDataPtr++;									// Get R
			G = *DACColourDataPtr++;									// Get G
			B = *DACColourDataPtr++;									// Get B
	
			if ((TTL & 0x08) && (R != 0 || G != 0 || B != 0)){			// Blanked?

				if (ShutterFlag == false){								// Shutter on?
					R = 90;
					G = 90;
					B = 90;
				}

				SetPixelV(DCHandle, ((((unsigned int)X * SCOPESIZE) / 65535) + 6), ((((unsigned int)(65535 - Y) * SCOPESIZE) / 65535) + 6), RGB(R,G,B));
			}
		}
	
	} else {

		Points = (POINT *)(DataPtr->SIMMemPtr + (2 * SIMBUFFSIZE));		// Point to Polyline points buffer
		
		FirstY = ((((unsigned int)(65535 - *DACDataPtr) * SCOPESIZE) / 65535) + 6);	// Get Y
		FirstX = ((((unsigned int)*(DACDataPtr + 1) * SCOPESIZE) / 65535) + 6);		// Get X
		TTL = *DACTTLDataPtr;														// Get TTL nibble

		R = *DACColourDataPtr;											// Get R
		G = *(DACColourDataPtr + 1);									// Get G
		B = *(DACColourDataPtr + 2);									// Get B
	
		if (TTL & 0x08){												// Blanked?
			if (ShutterFlag == false && (R != 0 || G != 0 || B != 0)){	// Shutter on?
				R = 90;
				G = 90;
				B = 90;
			}
		} else {
			R = 0;
			G = 0;
			B = 0;
		}

		FirstColour = LastColour = Colour = RGB(R,G,B);
		Pen = CreatePen(PS_SOLID, 0, Colour);

		if ((OrgObject = SelectObject(DCHandle, Pen)) != 0){
		
			if (FirstColour != 0)
				SetPixelV(DCHandle, FirstX, FirstY, FirstColour);		// Ensure first pixel is visible

			for (i = 0, a = 0; i < SIMPoints; i++, a++){
		
				Points[a].y = ((((unsigned int)(65535 - *DACDataPtr++) * SCOPESIZE) / 65535) + 6);	// Get Y
				Points[a].x = ((((unsigned int)*DACDataPtr++ * SCOPESIZE) / 65535) + 6);			// Get X
		
				if (i & 1)
					TTL = (*DACTTLDataPtr++ >> 4);						// Get TTL nibble
				else
					TTL = *DACTTLDataPtr;								// Get TTL nibble

				R = *DACColourDataPtr++;								// Get R
				G = *DACColourDataPtr++;								// Get G
				B = *DACColourDataPtr++;								// Get B

				if (TTL & 0x08){										// Blanked?
					if (ShutterFlag == false && (R != 0 || G != 0 || B != 0)){	// Shutter on?
						R = 90;
						G = 90;
						B = 90;
					}
				} else {
					R = 0;
					G = 0;
					B = 0;
				}

				Colour = RGB(R,G,B);

				if (a > 0 && Colour != 0 && Points[a].x == Points[a - 1].x && Points[a].y == Points[a - 1].y)	// Polyline doesn't draw single pixels
					 SetPixelV(DCHandle, Points[a].x, Points[a].y, Colour);

				if (Colour != LastColour){

					if (a > 1){

						if (LastColour != 0)
							Polyline(DCHandle, Points, a);

						Points[0].x = Points[a - 1].x;
						Points[0].y = Points[a - 1].y;
						Points[1].x = Points[a].x;
						Points[1].y = Points[a].y;
						a = 1;
					}

					Pen = CreatePen(PS_SOLID, 0, Colour);
					DeleteObject(SelectObject(DCHandle, Pen));
					LastColour = Colour;
				}
			}

			if (a > 1 && LastColour != 0)
				Polyline(DCHandle, Points, a);

			if (SIMPoints > 1 && FirstColour != 0 && DataPtr->IDNQueueCount == 0){	// Draw retrace line?

				if (FirstColour != LastColour){
					Pen = CreatePen(PS_SOLID, 0, FirstColour);
					DeleteObject(SelectObject(DCHandle, Pen));
					LastColour = FirstColour;
				}

				MoveToEx(DCHandle, Points[a - 1].x,  Points[a - 1].y, 0);
				LineTo(DCHandle, FirstX,  FirstY);
			}

			SelectObject(DCHandle, OrgObject);							// Restore original object
		}
						
		DeleteObject(Pen);
		
		if (ShowPointsFlag == true){									// Show "Fat" points?

			if (ShutterFlag == false)									// Shutter on?
				Colour = RGB(90,90,90);
			 else
				Colour = RGB(255,255,255);

			DACDataPtr = (unsigned short *)SIMBuffPtr;					// Point to sent buffer again

			for (i = SIMPoints; i != 0; i--){							// Draw "fat" points

				Y = ((((unsigned int)(65535 - *DACDataPtr++) * SCOPESIZE) / 65535) + 6);	// Get Y
				X = ((((unsigned int)*DACDataPtr++ * SCOPESIZE) / 65535) + 6);				// Get X

				SetPixelV(DCHandle, (X - 1), Y, Colour);
				SetPixelV(DCHandle, X, (Y + 1), Colour);
				SetPixelV(DCHandle, (X + 1), Y, Colour);
				SetPixelV(DCHandle, X, (Y - 1), Colour);
			}
		}
	}

	return;
}


LRESULT WINAPI SimWindowProc(HWND WinHandle, UINT Msg, WPARAM wParam, LPARAM lParam)
{
	switch (Msg){

	case WM_PAINT:
		{
			HDC DCHandle;
			PAINTSTRUCT Paint;

			if (GetUpdateRect(WinHandle, 0, false) != 0){
				if ((DCHandle = BeginPaint(WinHandle, &Paint)) != 0){
					DrawScope(WinHandle, DCHandle, (LASERFIREDATA *)SIMLfPtr, (unsigned char *)SIMBuffPtr, (unsigned int)SIMPoints);
					EndPaint(WinHandle, &Paint);
					SIMReady = true;									// Ready for next update
				}
			}

			return 0;
		}

		return 0;

	case WM_CLOSE:														// Request to close window

		DestroyWindow(WinHandle);
		return 0;

	case WM_DESTROY:

		SIMWindow = 0;
		SIMWindowFlag = false;
		ExitFlag = true;
		return 0;
	}

	return DefWindowProc(WinHandle, Msg, wParam, lParam);
}


void SIM(LASERFIREDATA *DataPtr)										// Synchronous function call
{
	static char SIMWinClassName[] = {"LfSIMxxx"};
	static bool InitFlag = false;
	static WNDCLASS SIMWinClass;
	HINSTANCE AppInstance;
	HWND ConWindow;
	RECT Rect;

	if (SIMWindowFlag == false)											// Just in case
		return;

	if (DataPtr == 0 && SIMWindow != 0)									// Close window?
		DestroyWindow(SIMWindow);
	
	if (InitFlag == false){
		InitFlag = true;												// Only once

		if ((AppInstance = GetModuleHandle(0)) == 0){
			ErrorCode = GetLastError();
			printf("SIM(): Can't get instance handle! (e=%d)\n", ErrorCode);
			PrintErrorCodeString();
			return;
		}

		if ((ConWindow = GetConsoleWindow()) == 0){
			printf("SIM(): Can't get console window handle!\n");
			return;
		}

		SIMWinClass.style = CS_OWNDC | CS_HREDRAW | CS_VREDRAW;			// Window style
		SIMWinClass.lpfnWndProc = SimWindowProc;						// Pointer to msg processing function
		SIMWinClass.cbClsExtra = 0;										// Extra bytes after the window class
		SIMWinClass.cbWndExtra = 0;										// Extra bytes after the window instance
		SIMWinClass.hInstance = AppInstance;							// The apps instance Handle
		SIMWinClass.hIcon = 0;											// The SIM icon
		SIMWinClass.hCursor = LoadCursor(0, IDC_ARROW);					// Use standard mouse pointer
		SIMWinClass.hbrBackground = CreateSolidBrush(DEFAULTBACKGROUNDCOLOUR);	// Background colour brush
		SIMWinClass.lpszMenuName = 0;									// Pointer to name of menu resource
		SIMWinClass.lpszClassName = SIMWinClassName;					// Pointer to class name

		RegisterClass(&SIMWinClass);

		if ((SIMWindow = CreateWindowEx(WS_EX_NOPARENTNOTIFY, SIMWinClassName, "IDN-Scope", WS_POPUP | WS_VISIBLE | WS_SYSMENU | WS_CAPTION,
				CW_USEDEFAULT, CW_USEDEFAULT, SCOPESIZE + 30, SCOPESIZE + 52, ConWindow, 0, AppInstance, 0)) == 0){
			ErrorCode = GetLastError();
			printf("SIM(): Can't create window! (e=%d)\n", ErrorCode);
			PrintErrorCodeString();
			return;
		}
	}
	
	if (SIMWindow == 0 || DataPtr == 0)									// Open/Close window only?
		return;

	if (DataPtr->UpdateScope == true || DataPtr->SIMFinalFlag == true){
		if (SIMReady == true){
			SIMLfPtr = DataPtr;											// Grab SIM data
			SIMPoints = DataPtr->SIMPoints;
			SIMBuffPtr = DataPtr->SIMBuffPtr;
			DataPtr->SIMBuffSwitch = (DataPtr->SIMBuffSwitch != false ? false : true);	// Switch SIM buffers
			DataPtr->SIMFinalFlag = false;								// No final update needed (yet)
		} else {
			DataPtr->SIMFinalFlag = true;								// Needs final update when ready
			return;
		}
	}

	if (SIMBuffPtr == 0 || SIMPoints == 0)								// Just in case
		return;

	SIMReady = false;													// Wait until scope drawn

	Rect.left = 5;														// Redraw scope always (in case of shutter changed)
	Rect.top = 5;
	Rect.right = SCOPESIZE + 8;
	Rect.bottom = SCOPESIZE + 8;
	InvalidateRect(SIMWindow, &Rect, false);
	return;
}


void CheckSIMMsg(void)
{
	MSG Msg;

	if (SIMWindow != 0){
		while (PeekMessage(&Msg, SIMWindow, 0, 0, PM_REMOVE) != 0){
			TranslateMessage(&Msg);
			DispatchMessage(&Msg);
//			printf("SIM(): Window message: %d\n", Msg.message);			// Debug only
		}
	}

	return;
}


void ShutterClose(LASERFIREDATA *DataPtr)
{
	DataPtr->ShutterFlag = false;


	// Add Shutter close here


	if (SIMWindowFlag == true)											// Update SIM window?
		SIM(DataPtr);

	return;
}


void ShutterOpen(LASERFIREDATA *DataPtr)
{
	DataPtr->ShutterFlag = true;


	// Add Shutter open here


	if (SIMWindowFlag == true)											// Update SIM window?
		SIM(DataPtr);

	return;
}


void IDNShowStats(LASERFIREDATA *Data)
{
	static MYTIMERRETURN LastTime = Data->LastTime;
	
	if (ShowStatusInfoFlag == true && (Data->LastTime - LastTime) > (TADJUST * 5)){

		LastTime = Data->LastTime;

		if (Data->IDNWaveformMode != Data->IDNLastWaveformMode || Data->ShutterFlag != Data->LastShutterFlag || Data->IDNFormat != Data->IDNLastFormat ||
			Data->PPS != Data->LastPPS || Data->Freq != Data->LastFreq || Data->TotalPoints != Data->LastTotalPoints || Data->IDNSpeedAdjust != Data->IDNLastSpeedAdjust ||
			Data->IDNQueueCount != Data->IDNLastQueueCount || Data->IDNQueueTarget != Data->IDNLastQueueTarget || Data->SleepError != Data->LastSleepError){

			Data->IDNLastWaveformMode = Data->IDNWaveformMode;
			Data->LastShutterFlag = Data->ShutterFlag;
			Data->IDNLastFormat = Data->IDNFormat;
			Data->LastPPS = Data->PPS;
			Data->LastFreq = Data->Freq;
			Data->LastTotalPoints = Data->TotalPoints;
			Data->IDNLastSpeedAdjust = Data->IDNSpeedAdjust;
			Data->IDNLastQueueCount = Data->IDNQueueCount;
			Data->IDNLastQueueTarget = Data->IDNQueueTarget;
			Data->LastSleepError = Data->SleepError;


/*			Using a single printf() for quicker screen update 

			"\033[s\033[H\033[1J\033[2K\033[1;32m" -> Save cursor position, moves cursor to home position, erase to beginning of screen, erase the entire line and set text to bold and colour green (do this at start of printf)
			"\033[2K"                              -> Erase the entire line (do this after each new line)
			"\033[0m\n\033[2K\n\033[2K\033[u"      -> Reset colour, erase the next two lines and restore the cursor position (do this at end of printf)
			"\033[31m"                             -> Red text (41 = red background)
			"\033[32m"                             -> Green text (42 = green background)
			"\033[33m"                             -> Yellow text (43 = yellow background)
			"\033[34m"                             -> Blue text (44 = blue background)
			"\033[35m"                             -> Magenta text (45 = magenta background)
			"\033[36m"                             -> Cyan text (46 = cyan background)
			"\033[5m"                              -> Blinking text
			"\033[25m"                             -> Turn off blinking text
			"\033[7m"                              -> Inverse text
			"\033[27m"                             -> Turn off inverse text
*/

			printf("\033[s\033[H\033[1J\033[2K\033[1;32mMode: \033[36m%s\n\033[2K\033[32mShutter: \033[33m%s\n\033[2K\033[32mFormat: \033[33m\"%s\" (%d bytes)\n\033[2K\033[32mQueued frames: \033[33m%d (%d)\n\033[2K\033[32mAuto speed adjust: \033[33m%d%%\n\033[2K\033[32mAvg sleep error: \033[33m%d us\n\033[2K\033[32mTotal points: \033[33m%d\n\033[2K\033[32mFrequency: \033[33m%d.%d Hz\n\033[2K\033[32mPoints/second: \033[33m%d\033[0m\n\033[2K\n\033[2K\033[u",
				(Data->IDNWaveformMode == false ? "Frame" : "Waveform"),
				((Data->ShutterFlag == false) ? "Closed" : "\033[5;7mOpen\033[25;27m"),
				(Data->IDNFormat == 7 ? "XY\033[31mr\033[32mg\033[34mb\033[33m" : (Data->IDNFormat == 8 ? "XY\033[31mr\033[32mg\033[34mb\033[33mi" : (Data->IDNFormat == 10 ? "XY\033[31mR\033[32mG\033[34mB\033[33m" : "XY\033[31mR\033[32mG\033[34mB\033[33mI1234"))),
				Data->IDNFormat, Data->IDNQueueCount, Data->IDNQueueTarget, Data->IDNSpeedAdjust, Data->SleepError, Data->TotalPoints, Data->Freq / 10, Data->Freq % 10, Data->PPS);
		}
	}
			
	return;
}


void usleep(unsigned int Time_us)
{
    static HANDLE Timer = 0;
	unsigned int Time_ms;
    LARGE_INTEGER Wait_ns;

	if (Time_us != 0){

		if (Timer == 0){
			if ((Timer = CreateWaitableTimer(0, true, 0)) == 0)
				goto FallBack;
		}
		
		Wait_ns.QuadPart = -(10 * (__int64)Time_us);					// Convert to 100 nanosecond interval, negative value indicates relative time
		
		if (SetWaitableTimer(Timer, &Wait_ns, 0, 0, 0, 0) == 0)
			goto FallBack;

		if (WaitForSingleObject(Timer, INFINITE) == WAIT_FAILED)
			goto FallBack;
		
	} else {

		if (Timer != 0){
			CloseHandle(Timer);
			Timer = 0;
		}
	}
			
	return;

FallBack:

//	printf("Sleep fall back used!\n");									// Debug only

	if ((Time_ms = (Time_us / 1000)) != 0)								// Fall back to standard sleep function
		Sleep(Time_ms);

	return;
}


void FreeMem(unsigned char **MemPtr)
{
	if (*MemPtr != 0){
		delete[] *MemPtr;
		*MemPtr = 0;
	}

	return;
}


DWORD WINAPI MainThread(LPVOID)
{
	unsigned char *IDNMemPtr = 0, *SIMMemPtr = 0;
	MYTIMERRETURN StartTime, CurrentTime, CurrentTimestamp = 0;
	unsigned long long SleepErrorCount = 0, AccumulatedSleepErrors = 0;
	unsigned int Size = 0, Sleep_us, MainLoopTime, CurrentDuration_us = 0, SyncedDuration_us = 0, i, Temp;
	bool RestoreTimer = false, NoSleepFlag = false;
	SOCKET IDNSocket, IDNTCPSocket, ActiveIDNSocket = 0;
	sockaddr_in IDNClientAddress;
	LASERFIREDATA LASERfIREData;
	IDNQUEUE IDNQueueData[IDNMAXQUEUE];
	RESPONSEDATA IDNResponse;


	/* Allocate memory */

	printf("Allocating buffer memory...\n");
	
	if ((IDNMemPtr = new unsigned char[IDNBUFFSIZE]) == 0){
		printf("Can't allocate IDN buffer memory! (%d bytes)\n", IDNBUFFSIZE);
		PrintErrorCodeString();
		goto ErrorExit;
	}

	LASERfIREData.IDNMemPtr = IDNMemPtr;
	
	if ((SIMMemPtr = new unsigned char[SIMBUFFTOTALSIZE]) == 0){
		printf("Can't allocate SIM buffer memory! (%d bytes)\n", (int)SIMBUFFTOTALSIZE);
		PrintErrorCodeString();
		FreeMem(&IDNMemPtr);
		goto ErrorExit;
	}

	LASERfIREData.SIMMemPtr = SIMMemPtr;
		
	printf("Total memory needed for buffers: %d bytes.\n", (int)(IDNBUFFSIZE + SIMBUFFTOTALSIZE));


	/* Initialise */

	printf("Initialising data...\n");
	LASERfIREData.IDNQueueData = IDNQueueData;							// Point to IDN queue data
	LASERfIREData.IDNResponseData = &IDNResponse;						// Point to IDN response data
	LASERfIREData.SIMBuffPtr = 0;										// No SIM() buffer yet
	LASERfIREData.SIMPoints = 0;										// No SIM() points yet
	LASERfIREData.SIMFinalFlag = false;									// No SIM() final update yet
	LASERfIREData.FrameTimestamp = 0;									// No timestamp yet
	LASERfIREData.SleepError = 0;										// No sleep error yet
	IDNResponse.IDNClientAddressPtr = &IDNClientAddress;
	IDNResponse.IDNSocketPtr = &IDNSocket;
	IDNResponse.ActiveIDNSocketPtr = &ActiveIDNSocket;
	IDNResponse.PacketCount = 0;
	IDNResponse.ClientGroup = 0;

	printf("Initialising IDN UDP server...\n");
	if ((IDNSocket = InitIDNServ()) == 0){
		PrintErrorCodeString();
		FreeMem(&SIMMemPtr);
		FreeMem(&IDNMemPtr);
		goto ErrorExit;
	}

	printf("Initialising IDN TCP server...\n");
	if ((IDNTCPSocket = InitIDNTCPServ()) == 0){
		PrintErrorCodeString();
		closesocket(IDNSocket);
		WSACleanup();
		FreeMem(&SIMMemPtr);
		FreeMem(&IDNMemPtr);
		goto ErrorExit;
	}

	if (ExtendedErrorMsgsFlag == true)
		printf("\033[1;34mExtended error and warning messages are enabled\033[0m.\n");

	if (timeBeginPeriod(2) == TIMERR_NOERROR)							// Increase timer resolution to reduce sleep errors and keep up with large IDN waveforms
		RestoreTimer = true;
	else
		printf("Warning, failed to increase timer resolution!\n");

	printf("IDN-Scope is up and running...\n\n");


	/* Main program loop here */

MainLoop:

	StartTime = MYTIMER();												// Time how long the main loop takes

	do {

		if ((Size = GetIDNData(IDNSocket, &IDNClientAddress, (IDNMemPtr + IDNINPUTBUFFOFFSET))) == 0)	// Check for IDN data on both protocols
			Size = GetIDNTCPData(IDNTCPSocket, &ActiveIDNSocket, &IDNClientAddress, (IDNMemPtr + IDNINPUTBUFFOFFSET), &NoSleepFlag);

		if (Size != 0){
	
			if (Size == (unsigned int)-1){
		
				Size = 0;												// Prevents while loop lockup
				PrintErrorCodeString();
		
			} else {
			
				if (IDNModeFlag == false){								// New connection?
					LASERfIREData.UpdateScope = false;					// Ignore engine update (if any)
					LASERfIREData.IDNFormat = 7;						// Set format to default (7 bytes)
					LASERfIREData.IDNCurrentChannelID = 0xff;			// Set channel closed
					LASERfIREData.IDNWaveformMode = false;				// Set mode to default
					LASERfIREData.IDNSyncPacketCountFlag = true;		// Resync packet count
					LASERfIREData.IDNGracefullyCloseShutterFlag = false;	// No gracefully close yet
					LASERfIREData.IDNFragmentedFlag = false;			// No fragments (yet)
					LASERfIREData.IDNSpeedAdjust = 100;					// Start with no speed adjust (100%)
					LASERfIREData.IDNQueueTarget = 0;					// Start with no queued target (yet)
					LASERfIREData.IDNQueueCount = 0;					// No queued frames
					for (i = 0; i < IDNMAXQUEUE; i++)					// Clear all queue slots
						IDNQueueData[i].Sequence = 0;
					LASERfIREData.IDNUnqueueFlag = false;				// Nothing to unqueue (yet)
					LASERfIREData.IDNBuffPtr = 0;						// Nothing to stream (yet)
					IDNModeFlag = true;									// Activate IDN mode
				}

				ProcessIDNData(&LASERfIREData, Size);					// Sets UpdateScope
			}
		}
	
		if (IDNModeFlag == true){										// IDN Mode?
			if (LASERfIREData.UpdateScope == true){						// Frame ready?
				if (IDNDACUpdate(&LASERfIREData) == true){				// Buffer free (sent)?
					if (SIMWindowFlag == true)							// Update SIM window?
						SIM(&LASERfIREData);
					if (LASERfIREData.IDNUnqueueFlag == true){			// Need to unqueue sent frame?
						LASERfIREData.IDNUnqueueFlag = false;			// Just once
						LASERfIREData.IDNQueueCount--;					// One less queued frame
						for (i = 0; i < IDNMAXQUEUE; i++)				// Update sequence order
							if (IDNQueueData[i].Sequence != 0)
								IDNQueueData[i].Sequence--;
					}
					if (LASERfIREData.ShutterFlag == false)				// Shutter already open?
						ShutterOpen(&LASERfIREData);					// Open shutter
					if (LASERfIREData.IDNQueueCount != 0){				// Any queued frames?
						for (i = 0; i < IDNMAXQUEUE; i++){				// Find next queued frame
							if (IDNQueueData[i].Sequence == 1){
								if ((Temp = ((LASERfIREData.IDNSpeedAdjust * IDNQueueData[i].PPS) / 100)) > IDNMAXDACPPS)	// Calc adjusted PPS 
									Temp = IDNMAXDACPPS;				// Cap PPS to hardware limit
								else
									if (Temp < MINPPS)
										Temp = MINPPS;
								LASERfIREData.PPS = Temp;				
								LASERfIREData.TotalPoints = IDNQueueData[i].Points;
								LASERfIREData.IDNChecksum = IDNQueueData[i].Checksum;
								LASERfIREData.IDNBuffPtr = (IDNMemPtr + IDNMAXSAMPLESIZE + (i * IDNQOUTBUFFSIZE));			// Point to queued DAC data
								LASERfIREData.IDNUnqueueFlag = true;	// Unqueue this frame
								LASERfIREData.Freq = CalcFreq(Temp, LASERfIREData.TotalPoints);								// Update Freq
								break;
							}
						}
						if (i == IDNMAXQUEUE){							// Just in case
							if (ExtendedErrorMsgsFlag == true)
								printf("Queued IDN frame data is corrupted - resetting queue (should never get here)!\n");
							LASERfIREData.IDNQueueCount = 0;			// No queued frames
							for (i = 0; i < IDNMAXQUEUE; i++)			// Clear all queue slots
								IDNQueueData[i].Sequence = 0;
							LASERfIREData.UpdateScope = false;			// All frames sent
						}

					} else {
						LASERfIREData.UpdateScope = false;				// All frames sent
					}
				}
			}
		}

	} while (Size != 0);												// Get all available datagrams

	if (IDNModeFlag == true){
		LASERfIREData.LastTime = MYTIMER();								// Get current time
		if (CurrentDuration_us == 0 && LASERfIREData.FrameTimestamp == CurrentTimestamp && LASERfIREData.UpdateScope == false && LASERfIREData.ShutterFlag == true &&	// Need to close the shutter?
				(LASERfIREData.IDNWaveformMode == true || LASERfIREData.IDNGracefullyCloseShutterFlag == true)){	
			if (IDNDACUpdate(0) == true){								// Buffer free?
				ShutterClose(&LASERfIREData);							// Close shutter
				LASERfIREData.IDNGracefullyCloseShutterFlag = false;	// Only once
				if (ExtendedErrorMsgsFlag == true && LASERfIREData.IDNWaveformMode == true)
					printf("Warning, IDN buffer underrun on waveform mode - closing shutter...\n");
			}
		}
		IDNShowStats(&LASERfIREData);									// Print stats
		if (LASERfIREData.LastTime >= LASERfIREData.IDNTimeoutTimestamp){	// Check IDN connection timeout
			IDNModeFlag = false;
			LASERfIREData.IDNQueueCount = 0;							// No queued frames - let SIM() draw retrace
			LASERfIREData.UpdateScope = false;							// Ignore update (if any)
			ShutterClose(&LASERfIREData);								// Close shutter (if not already)
			CloseLastSocket(&ActiveIDNSocket);							// Close last active IDN TCP socket
			printf("IDN connection timeout.\n");
			LASERfIREData.LastPPS = -1;									// Force status update because status may have scrolled because of printf()
		}
	}

	if (SIMWindowFlag == true && LASERfIREData.SIMFinalFlag == true)	// Update SIM window?
		SIM(&LASERfIREData);


	/* Main loop execution time - includes DACUpdate() time */

	CurrentTime = MYTIMER();
	MainLoopTime = (unsigned int)(CurrentTime - StartTime);				// Dont care about overflow


	/* Sleep management - let the CPU cool down */
	
	if (IDNModeFlag == true && CurrentDuration_us == 0 && LASERfIREData.FrameTimestamp == CurrentTimestamp && LASERfIREData.UpdateScope == false && LASERfIREData.ShutterFlag == true && LASERfIREData.IDNWaveformMode == true)
		Sleep_us = 0;													// No sleeping as buffer underrun possible!
	else
		Sleep_us = SLEEPTIME;											// Use default sleep

	if (LASERfIREData.FrameTimestamp != CurrentTimestamp){				// New timestamp?
		CurrentTimestamp = LASERfIREData.FrameTimestamp;				// Update current timestamp
		SyncedDuration_us = CurrentDuration_us;							// Resync to previous sent buffer
		CurrentDuration_us = LASERfIREData.FramePeriod_us;				// New current duration
	}

	Temp = (unsigned int)(CurrentTime - CurrentTimestamp);

	if (Temp > (CurrentDuration_us + SyncedDuration_us))				// When not streaming and idle
		CurrentDuration_us = 0;											// Frame expired
		
	if (SyncedDuration_us != 0){										// Got sync yet?
		if (Temp >= SyncedDuration_us){
			SyncedDuration_us = 0;										// Frame expired
			Sleep_us = 0;												// Skip this sleep cycle to update scope
		} else {
			Temp = (SyncedDuration_us - Temp);							// Calc remaining time
			if (Sleep_us > Temp)										// Cap sleep time to remaining frame time
				Sleep_us = Temp;
		}
	}
		
	if (NoSleepFlag == false && Sleep_us != 0){

		if (LASERfIREData.SleepError < (SLEEPTIME / 2))					// Just in case (else will never sleep and never get accurate average)
			Temp = (MainLoopTime + LASERfIREData.SleepError);			// Calc sleep adjustment (MainLoopTime = microseconds)
		else
			Temp = MainLoopTime;

		if (Sleep_us > Temp){											// Enough time to sleep?
			Sleep_us -= Temp;											// Adjust sleep time
			StartTime = CurrentTime;									// Sleep start time 
			usleep(Sleep_us);		
			CurrentTime = MYTIMER();									// Update current time
			
			/* Calc average sleep error - gets more accurate over time */

			Temp = (unsigned int)(CurrentTime - StartTime);				// Calc actual sleep time (microseconds)
			if (Temp > Sleep_us)										// Slept longer than expected?
				AccumulatedSleepErrors += (Temp - Sleep_us);
			else
				if (Temp < Sleep_us){									// Slept less than expected?
					if (AccumulatedSleepErrors > (Sleep_us - Temp))
						AccumulatedSleepErrors -= (Sleep_us - Temp);
					else
						AccumulatedSleepErrors = 0;
				}

			LASERfIREData.SleepError = (unsigned int)(AccumulatedSleepErrors / ++SleepErrorCount);	// Calc new average sleep error 
		}
					
	} else {
		
		NoSleepFlag = false;											// Just once
	}

	/* Do it all again and again */

	if (ExitFlag == false)												// Sim() window closed?
		goto MainLoop;

	if (RestoreTimer == true){											// Restore timer resolution?
		RestoreTimer = false;											// Just once
		timeEndPeriod(2);
	}


	/* Exit */

	CloseLastSocket(&ActiveIDNSocket);
	shutdown(IDNTCPSocket, SD_BOTH);
	closesocket(IDNTCPSocket);
	closesocket(IDNSocket);
	WSACleanup();
	FreeMem(&SIMMemPtr);
	FreeMem(&IDNMemPtr);

	ThreadExitStatus = 1;
	return 0;


ErrorExit:
	ThreadExitStatus = -1;
	return 0;
}


bool StartMainThread(void)
{
	DWORD ThreadID;
	HANDLE ThreadHandle;
	
	if ((ThreadHandle = CreateThread(0, 0, MainThread, 0, 0, &ThreadID)) == 0){
		ErrorCode = GetLastError();
		return false;
	}

	CloseHandle(ThreadHandle);
	return true;
}


int main(int argc, char *argv[])										// Entry point
{
	unsigned char *TempPtr;
	unsigned int i, Temp;


	/* Start message */

	printf("\n\n");
	printf("\033[2J\033[1;33;44m*********************************\033[0m\n");	// Erase all, Bold, Yellow, Blue background
	printf("\033[1;33;44m*                               *\033[0m\n");
	printf("\033[1;33;44m*     IDN-Scope for Windows     *\033[0m\n");
	printf("\033[1;33;44m*                               *\033[0m\n");
	printf("\033[1;33;44m*   Version 1.07  18-Aug-2026   *\033[0m\n");
	printf("\033[1;33;44m*                               *\033[0m\n");
	printf("\033[1;33;44m*  (c) 2026 by Anthony Barrett  *\033[0m\n");
	printf("\033[1;33;44m*                               *\033[0m\n");
	printf("\033[1;33;44m*********************************\033[0m\n");
	printf("\n\n");

	printf("Maximum points: %d\n\n", MaxPnts);
	
	
	/* Check for command arguments */

	if (argc > 1){

		i = 1;

		while (i < (unsigned int)argc){

			TempPtr = (unsigned char *)argv[i++];

			do {
				switch (*TempPtr){

				case 'z':												// IDN network port?
				case 'Z':

					TempPtr++;

					Temp = GetNum(&TempPtr, TempPtr + 10);

					if (Temp > MAXNETWORKPORT || Temp < MINNETWORKPORT){
						printf("\nIDN network port value out of range (1024-49151).\n\n");
						return 0;
					}

					IDNPort = Temp;
					TempPtr--;
					break;

				case 'k':												// Draw between points?
				case 'K':
					DrawBetweenPointsFlag = true;
					SIMWindowFlag = true;								// Force SIM window
					break;

				case 'v':												// Show points?
				case 'V':
					ShowPointsFlag = true;
					DrawBetweenPointsFlag = true;						// Force draw between points
					SIMWindowFlag = true;								// Force SIM window
					break;

				case 'i':												// Status info?
				case 'I':
					ShowStatusInfoFlag = true;
					break;

				case 'c':												// Connection info?
				case 'C':
					ShowConnectionInfoFlag = true;
					break;

				case 'e':												// Extended error msgs?
				case 'E':
					ExtendedErrorMsgsFlag = true;
					break;

				default:

			       	if (*TempPtr != '-' && *TempPtr != '?'){
			       		printf("\nUnknown argument: \"%s\"!\n", TempPtr);
					}
					
					printf("\n");
					printf("Usage: IDN-Scope [z#][k][v][i][c][e]\n");
					printf("\n");
					printf(" 'z#' = IDN server network port (1024-49151).\n");
					printf("  'k' = Draw between points on simulator window.\n");
					printf("  'v' = Show \"fat\" points when drawing between points on simulator window.\n");
					printf("  'i' = Output status information to console.\n");
					printf("  'c' = Output connection information to console (debug use).\n");
					printf("  'e' = Output error and warning messages to console (debug use).\n");
					printf("\n");
					return 0;
				}

			} while (*++TempPtr != '\0');
		}
	}


	/* Test high resolution timer */

	if (MYTIMER() == 0){
		ErrorCode = GetLastError();
		printf("Can't get high resolution timer ticks! (e=%d)\n", ErrorCode);
		PrintErrorCodeString();
		goto Exit;
	}


	/* Start simulator */

	printf("Starting simulator...\n");
	SIM(0);


	/* Start main thread */

	printf("Starting main thread...\n");
	if (StartMainThread() == false){
		printf("Can't start main thread! (e=%d)\n", ErrorCode);
		PrintErrorCodeString();
		SIM(0);															// Close SIM window
		goto Exit;
	}


	/* Main program loop here */

	while (ThreadExitStatus == 0){
		CheckSIMMsg();
//		Sleep(SLEEPTIME / 1000);
		Sleep(1);
	}


	/* Exit */

	SIM(0);																// Close SIM window
	usleep(0);															// Close timer

	if (ThreadExitStatus == -1)
		goto Exit;

	CheckSIMMsg();
	printf("\n\nClean exit from IDN-Scope.\n\n");
	return 0;


Exit:
	printf("\n");
	for (i = 5; i != 0; i--){
		CheckSIMMsg();
		if (i > 1)
			printf("Exiting in %d seconds...  \r", i);
		else
			printf("Exiting in %d second...   \r", i);
		Sleep(1000);													// Wait 1 second
	}
	printf("Clean exit from IDN-Scope.\n\n");
	return 0;
}
