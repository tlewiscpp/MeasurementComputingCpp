//
// Created by Pingu on 2/15/2018.
//

#include "../USB_IO_Base.hpp"

#include "common/cbw.h"
#include <cstring>
#include <string>
#include <sstream>

#define NAME_BUFFER_MAX 1024

namespace {
    template <typename T> static inline std::string toStdString(const T &t) { return dynamic_cast<std::ostringstream &>(std::ostringstream{} << t).str(); }
}

namespace MeasurementComputingCpp {

std::string USB_IO_Base::getBoardName(unsigned int boardNumber) {
	char nameBuffer[NAME_BUFFER_MAX];
	memset(nameBuffer, '\0', NAME_BUFFER_MAX);
	auto result = cbGetBoardName(static_cast<int>(boardNumber), nameBuffer);
	if (result < 0) {
		throw std::runtime_error("USB_IO_Base::getBoardName(): cbGetBoardName returned " + toStdString(result) + " (" + this->getErrorString(result) + ")");
	}
}

std::string USB_1024LS::serialNumber() const {
	if (!this->m_serialNumber.empty()) {
    return this->m_serialNumber;
	}

	int EXTCCONV cbGetConfigString(int InfoType, int BoardNum, int DevNum,
		int ConfigItem, char* ConfigVal, int* maxConfigLen);


	char tempSerialNumber[SERIAL_NUMBER_BUFFER_1024LS];
	memset(tempSerialNumber, '\0', SERIAL_NUMBER_BUFFER_1024LS);

	auto max = SERIAL_NUMBER_BUFFER_1024LS;

	auto result = cbGetConfigString(BOARDINFO, this->m_boardNumber, 0, BISERIALNUM, tempSerialNumber, &max);
	if (result != NOERRORS) {
		throw std::runtime_error("USB_1024LS::serialNumber(): cbGetConfigString returned " + toStdString(result) + " (" + getErrorString(result) + ")");
	}
	this->m_serialNumber = std::string{tempSerialNumber};
	return this->m_serialNumber;
}

std::string USB_IO_Base::getErrorString(unsigned int errorCode) {
	switch (errorCode) {
	case (NOERRORS):
		return "No error occurred";
	case (BADBOARD):
		return "Invalid board number specified";
	case (DEADDIGITALDEV):
		return "Digital I/O device is not responding ";
	case (DEADCOUNTERDEV):
		return "Counter I/O device is not responding";
	case (DEADDADEV):
		return "D/A is not responding";
	case (DEADADDEV):
		return "A/D is not responding";
	case (NOTDIGITALCONF):
		return "Specified board does not have digital I/O";
	case (NOTCOUNTERCONF):
		return "Specified board does not have a counter";
	case (NOTDACONF):
		return "Specified board is does not have D/A";
	case (NOTADCONF):
		return "Specified board does not have A/D";
	case (NOTMUXCONF):
		return "Specified board does not have thermocouple inputs";
	case (BADPORTNUM):
		return "Invalid port number specified";
	case (BADCOUNTERDEVNUM):
		return "Invalid counter device";
	case (BADDADEVNUM):
		return "Invalid D/A device";
	case (BADSAMPLEMODE):
		return "Invalid sampling mode option specified";
	case (BADINT):
		return "Board configured for invalid interrupt level";
	case (BADADCHAN):
		return "Invalid A/D channel Specified";
	case (BADCOUNT):
		return "Invalid count specified";
	case (BADCNTRCONFIG):
		return "invalid counter configuration specified";
	case (BADDAVAL):
		return "Invalid D/A output value specified";
	case (BADDACHAN):
		return "Invalid D/A channel specified";
	case (ALREADYACTIVE):
		return "A background process is already in progress";
	case (PAGEOVERRUN):
		return "DMA transfer crossed page boundary, may have gaps in data";
	case (BADRATE):
		return "Inavlid sampling rate specified";
	case (COMPATMODE):
		return "Board switches set for "compatible" mode";
	case (TRIGSTATE):
		return "Incorrect intial trigger state D0 must=TTL low)";
	case (ADSTATUSHUNG):
		return "A/D is not responding";
	case (TOOFEW):
		return "Too few samples before trigger occurred";
	case (OVERRUN):
		return "Data lost due to overrun, rate too high";
	case (BADRANGE):
		return "Invalid range specified";
	case (NOPROGGAIN):
		return "Board does not have programmable gain";
	case (BADFILENAME):
		return "Not a legal DOS filename";
	case (DISKISFULL):
		return "Couldn't complete, disk is full";
	case (COMPATWARN):
		return "Board is in compatible mode, so DMA will be used";
	case (BADPOINTER):
		return "Invalid pointer (NULL)";
	case (TOOMANYGAINS):
		return "Too many gains";
	case (RATEWARNING):
		return "Rate may be too high for interrupt I/O";
	case (CONVERTDMA):
		return "CONVERTDATA cannot be used with DMA I/O";
	case (DTCONNECTERR):
		return "Board doesn't have DT Connect";
	case (FORECONTINUOUS):
		return "CONTINUOUS can only be used with BACKGROUND";
	case (BADBOARDTYPE):
		return "This function can not be used with this board";
	case (WRONGDIGCONFIG):
		return "Digital I/O is configured incorrectly";
	case (NOTCONFIGURABLE):
		return "Digital port is not configurable";
	case (BADPORTCONFIG):
		return "Invalid port configuration specified";
	case (BADFIRSTPOINT):
		return "First point argument is not valid";
	case (ENDOFFILE):
		return "Attempted to read past end of file";
	case (NOT8254CTR):
		return "This board does not have an 8254 counter";
	case (NOT9513CTR):
		return "This board does not have a 9513 counter";
	case (BADTRIGTYPE):
		return "Invalid trigger type";
	case (BADTRIGVALUE):
		return "Invalid trigger value";
	case (BADOPTION):
		return "Invalid option specified for this function";
	case (BADPRETRIGCOUNT):
		return "Invalid pre-trigger count sepcified";
	case (BADDIVIDER):
		return "Invalid fout divider value";
	case (BADSOURCE):
		return "Invalid source value ";
	case (BADCOMPARE):
		return "Invalid compare value";
	case (BADTIMEOFDAY):
		return "Invalid time of day value";
	case (BADGATEINTERVAL):
		return "Invalid gate interval value";
	case (BADGATECNTRL):
		return "Invalid gate control value";
	case (BADCOUNTEREDGE):
		return "Invalid counter edge value";
	case (BADSPCLGATE):
		return "Invalid special gate value";
	case (BADRELOAD):
		return "Invalid reload value";
	case (BADRECYCLEFLAG):
		return "Invalid recycle flag value";
	case (BADBCDFLAG):
		return "Invalid BCD flag value";
	case (BADDIRECTION):
		return "Invalid count direction value";
	case (BADOUTCONTROL):
		return "Invalid output control value";
	case (BADBITNUMBER):
		return "Invalid bit number";
	case (NONEENABLED):
		return "None of the counter channels are enabled";
	case (BADCTRCONTROL):
		return "Element of control array not ENABLED/DISABLED";
	case (BADEXPCHAN):
		return "Invalid EXP channel";
	case (WRONGADRANGE):
		return "Wrong A/D range selected for cbtherm";
	case (OUTOFRANGE):
		return "Temperature input is out of range";
	case (BADTEMPSCALE):
		return "Invalid temperate scale";
	case (BADERRCODE):
		return "Invalid error code specified";
	case (NOQUEUE):
		return "Specified board does not have chan/gain queue";
	case (CONTINUOUSCOUNT):
		return "CONTINUOUS can not be used with this count value";
	case (UNDERRUN):
		return "D/A FIFO hit empty while doing output";
	case (BADMEMMODE):
		return "Invalid memory mode specified";
	case (FREQOVERRUN):
		return "Measured frequency too high for gating interval";
	case (NOCJCCHAN):
		return "Board does not have CJC chan configured";
	case (BADCHIPNUM):
		return "Invalid chip number used with cbC9513Init";
	case (DIGNOTENABLED):
		return "Digital I/O not enabled";
	case (CONVERT16BITS):
		return "CONVERT option not allowed with 16 bit A/D";
	case (NOMEMBOARD):
		return "EXTMEMORY option requires memory board";
	case (DTACTIVE):
		return "Memory I/O while DT Active";
	case (NOTMEMCONF):
		return "Specified board is not a memory board";
	case (ODDCHAN):
		return "First chan in queue can not be odd";
	case (CTRNOINIT):
		return "Counter was not initialized";
	case (NOT8536CTR):
		return "Specified counter is not an 8536";
	case (FREERUNNING):
		return "A/D sampling is not timed";
	case (INTERRUPTED):
		return "Operation interrupted with CTRL-C";
	case (NOSELECTORS):
		return "Selector could not be allocated";
	case (NOBURSTMODE):
		return "Burst mode is not supported on this board";
	case (NOTWINDOWSFUNC):
		return "This function not available in Windows lib";
	case (NOTSIMULCONF):
		return "Not configured for simultaneous update";
	case (EVENODDMISMATCH):
		return "Even channel in odd slot in the queue";
	case (M1RATEWARNING):
		return "DAS16/M1 sample rate too fast";
	case (NOTRS485):
		return "Board is not an RS-485 board";
	case (NOTDOSFUNC):
		return "This function not avaliable in DOS";
	case (RANGEMISMATCH):
		return "Unipolar and Bipolar can not be used together in A/D que";
	case (CLOCKTOOSLOW):
		return "Sample rate too fast for clock jumper setting";
	case (BADCALFACTORS):
		return "Cal factors were out of expected range of values";
	case (BADCONFIGTYPE):
		return "Invalid configuration type information requested";
	case (BADCONFIGITEM):
		return "Invalid configuration item specified";
	case (NOPCMCIABOARD):
		return "Can't acces PCMCIA board";
	case (NOBACKGROUND):
		return "Board does not support background I/O";
	case (STRINGTOOSHORT):
		return "String passed to cbGetBoardName is to short";
	case (CONVERTEXTMEM):
		return "Convert data option not allowed with external memory";
	case (BADEUADD):
		return "e_ToEngUnits addition error";
	case (DAS16JRRATEWARNING):
		return "use 10 MHz clock for rates > 125KHz";
	case (DAS08TOOLOWRATE):
		return "DAS08 rate set too low for AInScan warning";
	case (AMBIGSENSORONGP):
		return "more than one sensor type defined for EXP-GP";
	case (NOSENSORTYPEONGP):
		return "no sensor type defined for EXP-GP";
	case (NOCONVERSIONNEEDED):
		return "12 bit board without chan tags - converted in ISR";
	case (NOEXTCONTINUOUS):
		return "External memory cannot be used in CONTINUOUS mode";
	case (INVALIDPRETRIGCONVERT):
		return "cbAConvertPretrigData was called after failure in cbAPretrig";
	case (BADCTRREG):
		return "bad arg to CLoad for 9513";
	case (BADTRIGTHRESHOLD):
		return "Invalid trigger threshold specified in cbSetTrigger";
	case (BADPCMSLOTREF):
		return "No PCM card in specified slot";
	case (AMBIGPCMSLOTREF):
		return "More than one MCC PCM card in slot";
	case (BADSENSORTYPE):
		return "Bad sensor type selected in Instacal";
	case (DELBOARDNOTEXIST):
		return "tried to delete board number which doesn't exist";
	case (NOBOARDNAMEFILE):
		return "board name file not found";
	case (CFGFILENOTFOUND):
		return "configuration file not found";
	case (NOVDDINSTALLED):
		return "CBUL.386 device driver not installed";
	case (NOWINDOWSMEMORY):
		return "No Windows memory available";
	case (OUTOFDOSMEMORY):
		return "ISR data struct alloc failure";
	case (OBSOLETEOPTION):
		return "Obsolete option for cbGetConfig/cbSetConfig";
	case (NOPCMREGKEY):
		return "No registry entry for this PCMCIA board";
	case (NOCBUL32SYS):
		return "CBUL32.SYS device driver is not loaded";
	case (NODMAMEMORY):
		return "No DMA buffer available to device driver";
	case (IRQNOTAVAILABLE):
		return "IRQ in being used by another device";
	case (NOT7266CTR):
		return "This board does not have an LS7266 counter";
	case (BADQUADRATURE):
		return "Invalid quadrature specified";
	case (BADCOUNTMODE):
		return "Invalid counting mode specified";
	case (BADENCODING):
		return "Invalid data encoding specified";
	case (BADINDEXMODE):
		return "Invalid index mode specified";
	case (BADINVERTINDEX):
		return "Invalid invert index specified";
	case (BADFLAGPINS):
		return "Invalid flag pins specified";
	case (NOCTRSTATUS):
		return "This board does not support cbCStatus()";
	case (NOGATEALLOWED):
		return "Gating and indexing not allowed simultaneously";
	case (NOINDEXALLOWED):
		return "Indexing not allowed in non-quadratue mode";
	case (OPENCONNECTION):
		return "Temperature input has open connection";
	case (BMCONTINUOUSCOUNT):
		return "Count must be integer multiple of packetsize for recycle mode.";
	case (BADCALLBACKFUNC):
		return "Invalid pointer to callback function passed as arg";
	case (MBUSINUSE):
		return "MetraBus in use";
	case (MBUSNOCTLR):
		return "MetraBus I/O card has no configured controller card";
	case (BADEVENTTYPE):
		return "Invalid event type specified for this board.";
	case (ALREADYENABLED):
		return "An event handler has already been enabled for this event type";
	case (BADEVENTSIZE):
		return "Invalid event count specified.";
	case (CANTINSTALLEVENT):
		return "Unable to install event handler";
	case (BADBUFFERSIZE):
		return "Buffer is too small for operation";
	case (BADAIMODE):
		return "Invalid analog input mode(RSE, NRSE, or DIFF)";
	case (BADSIGNAL):
		return "Invalid signal type specified.";
	case (BADCONNECTION):
		return "Invalid connection specified.";
	case (BADINDEX):
		return "Invalid index specified, or reached end of internal connection list.";
	case (NOCONNECTION):
		return "No connection is assigned to specified signal.";
	case (BADBURSTIOCOUNT):
		return "Count cannot be greater than the FIFO size for BURSTIO mode.";
	case (DEADDEV):
		return "Device has stopped responding. Please check connections.";
	default:
		return "Unknown Error";
	}
}

} //namespace MeasurementComputingCpp