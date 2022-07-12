#include <epicsEvent.h>
#include "ADDriver.h"

#include "cordef.h"
#include "gevapi.h"

#define DRIVER_VERSION      1
#define DRIVER_REVISION     4
#define DRIVER_MODIFICATION 1

#define MAX_CAMERAS 1
#define NUM_BUFFERS 32

#define CAMERA_RESET_US 66636 // See command ref rev9 pg 5


class epicsShareClass shadobox : public ADDriver {
public:
    shadobox(const char *portName, char* serial_number,
                int maxBuffers, size_t maxMemory,
                int priority, int stackSize);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual void report(FILE *fp, int details);
	void mainDataTask();
	void imageGrabTask(GEV_BUFFER_OBJECT *img);
	void tempReadTask();
	void shutdown(void);

protected:
	int ShadFullWell;
    #define FIRST_SHAD_PARAM ShadFullWell
	int ShadGain;
	int ShadTrigMode;
	int ShadRetries;
	int ShadReadOutMode;
	int ShadTurbo;
	int ShadFFCEnable;
	int ShadFFCActive;
	int ShadDPCEnable;
	int ShadDPCActive;
	#define LAST_SHAD_PARAM ShadDPCActive

private:
    /* These are the methods that are new to this class */
	int grabImage(void);
	int setCameraFeature(std::string feature, unsigned int value);
	int setCameraFeature(std::string feature, int value);
	int setCameraFeature(std::string feature, double value);
	int setCameraFeature(std::string feature, std::string value);
	asynStatus disconnectCamera(void);
	int connectCamera(void);
	void updateSensorSize(void);
	int exposureTimeSecondsToClockCycles(double expTimeSeconds);
	double exposureTimeClockCyclesToSeconds(int expTimeClockCycles);
	void reconnectCamera();

    /* Our data */
    epicsEventId startEventId_;
    epicsEventId stopEventId_;
    NDArray *pRaw_;
	char* serial_number;
	int exiting_ = 0;

	GEV_DEVICE_INTERFACE telCamInt_[MAX_CAMERAS] = {0};
	GEV_CAMERA_HANDLE telCamHandle_ = NULL;
	GEV_CAMERA_OPTIONS telCamOpt_ = {0};
	GEVLIB_CONFIG_OPTIONS telLibOpt_ = {0};

	int numBuffers = NUM_BUFFERS;
	unsigned char* bufAddress[NUM_BUFFERS];
	int bufSize;

};

#define ShadFullWellString			"SHAD_FULL_WELL"
#define ShadGainString				"SHAD_GAIN"
#define ShadTrigModeString          "SHAD_TRIG_MODE"
#define ShadRetriesString           "SHAD_RETRIES"
#define ShadReadOutModeString		"SHAD_READOUT_MODE"
#define ShadTurboString				"SHAD_TURBO"
#define ShadFFCEnableString			"SHAD_FFC_ENABLE"
#define ShadFFCActiveString			"SHAD_FFC_ACTIVE"
#define ShadDPCEnableString			"SHAD_DPC_ENABLE"
#define ShadDPCActiveString			"SHAD_DPC_ACTIVE"
