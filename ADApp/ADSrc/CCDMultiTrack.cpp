/**
 * Area Detector class enabling multi-ROI driver for the Andor CCD.
 * This class is used by CCD camera modules that permit multiple regions-of-interest.
 *
 * Multi-ROI is typically used for multi-track spectrocopy application.
 *
 * There are 3 use cases:
 *    1 - The user sets only the track start array.
 *        This provides a single height track at those positions.
 *    2 - The user sets the start and end tracks arrays.
 *        This provides a fully-binned track between the start and end positions.
 *    3 - The user provides start, end and binning values.
 *        This provides (a less than fully binned) track between the start and end positions.
 *
 * Based on previous work by Peter Heesterman
 *
 * @author Colin Hogben,
 * @date Sep 2021
 *
 */

#include "CCDMultiTrack.h"

/* define C99 standard __func__ to come from MS's __FUNCTION__ */
#if defined ( _MSC_VER )
#define __func__ __FUNCTION__
#endif

/** Parameter strings to tie into asyn records */
#define CCDMultiTrackStartString "CCD_MULTI_TRACK_START"
#define CCDMultiTrackEndString "CCD_MULTI_TRACK_END"
#define CCDMultiTrackBinString "CCD_MULTI_TRACK_BIN"

static const char* const driverName = "CCDMultiTrack";

#define TRACE(fmt,...) asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s:%s " fmt "\n", driverName, __func__, __VA_ARGS__)

CCDMultiTrack::CCDMultiTrack(asynPortDriver* asynPortDriver)
{
    mPortDriver = asynPortDriver;
    /* Semi-sensible value for old AD instances which don't call setMaxSize() */
    mMaxSizeY = 5000;
    /* Create parameters and get indices */
    asynPortDriver->createParam(CCDMultiTrackStartString, asynParamInt32Array, &mCCDMultiTrackStart);
    asynPortDriver->createParam(CCDMultiTrackEndString, asynParamInt32Array, &mCCDMultiTrackEnd);
    asynPortDriver->createParam(CCDMultiTrackBinString, asynParamInt32Array, &mCCDMultiTrackBin);
}

/** Set size of CCD */
void CCDMultiTrack::setMaxSize(asynUser *pasynUser, size_t maxSizeY) {
    TRACE("maxSizeY = %zd", maxSizeY);
    mMaxSizeY = maxSizeY;
    _validate(pasynUser);
}

/** Handle array from user */
asynStatus CCDMultiTrack::writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements)
{
    int function = pasynUser->reason;   // Parameter index
    asynStatus status = asynSuccess;

    std::vector<int> valueArray(nElements);
    for (size_t i = 0; i < nElements; i++) {
        valueArray[i] = value[i];
    }

    std::vector<int> *puserArray = 0;
    if (function == mCCDMultiTrackStart) {
        TRACE("setting Start[:%zd]", nElements);
        puserArray = &mUserStart;
    }
    else if (function == mCCDMultiTrackEnd) {
        TRACE("setting End[:%zd]", nElements);
        puserArray = &mUserEnd;
    }
    else if (function == mCCDMultiTrackBin) {
        TRACE("setting Bin[:%zd]", nElements);
        puserArray = &mUserBin;
    }
    else {
        TRACE("Unknown reason %d", function);
        status = asynError;             // Not our parameter
    }

    if (puserArray) {
        if (! (valueArray == *puserArray)) {
            *puserArray = valueArray;
            _validate(pasynUser);

            for (unsigned m = 0; m < mMessages.size(); m++) {
                TRACE("message %s", mMessages[m].c_str());
                asynPrint(pasynUser, ASYN_TRACE_WARNING,
                        "CCDMultiTrack: %s\n", mMessages[m].c_str());
            }
        }
    }

    return status;
}

void CCDMultiTrack::_validate(asynUser *pasynUser)
{
    /* Call the virtual method to check/adjust; result in mValid */
    validate(pasynUser);

    /* Write adjusted values back */
    std::vector<int> validStart(size());
    std::vector<int> validEnd(size());
    std::vector<int> validBin(size());
    for (unsigned i = 0; i < size(); i++) {
        validStart[i] = mValid[i].offset;
        validEnd[i] = mValid[i].offset + mValid[i].size - 1;
        validBin[i] = mValid[i].binning;
    }
    mPortDriver->doCallbacksInt32Array(&validStart[0], validStart.size(),
            mCCDMultiTrackStart, 0);
    mPortDriver->doCallbacksInt32Array(&validEnd[0], validEnd.size(),
            mCCDMultiTrackEnd, 0);
    mPortDriver->doCallbacksInt32Array(&validBin[0], validBin.size(),
            mCCDMultiTrackBin, 0);
}

/** Helper for validate() */
void CCDMultiTrack::addMessage(const char *fmt, ...)
{
    char buf[120];
    va_list ap;
    va_start(ap, fmt);
    epicsVsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    mMessages.push_back(buf);
}

/** Derive valid regions from user settings.
 * Record messages about any invalid values, adjusting where necessary.
 */
void CCDMultiTrack::validate(asynUser *pasynUser)
{
    std::vector<NDDimension_t> regions;
    mMessages.resize(0);

    size_t numRegions = mUserStart.size();
    if (numRegions > mMaxSizeY) {
        addMessage("More tracks (%d) than Y pixels", (int) numRegions);
        numRegions = mMaxSizeY;
    }
    regions.resize(numRegions);

    for (unsigned i = 0; i < numRegions; i++) {
        NDDimension_t &region = regions[i];
        int prevEnd = (i == 0) ? 0 : (regions[i-1].offset + regions[i-1].size);
        if (mUserStart[i] < 0) {
            addMessage("Track %d start (%d) less than 0",
                    i+1, mUserStart[i]);
            region.offset = 0;
        } else if (i > 0 && mUserStart[i] < prevEnd) {
            addMessage("Track %d start (%d) before end of previous (%d)",
                    i+1, mUserStart[i], prevEnd);
            region.offset = prevEnd;
        } else if (mUserStart[i] > (int) mMaxSizeY - 1) {
            addMessage("Track %d start (%d) beyond last row (%d)",
                    i+1, mUserStart[i], (int) mMaxSizeY - 1);
            region.offset = mMaxSizeY - (numRegions - i);
        } else if (mUserStart[i] > (int) (mMaxSizeY - (numRegions - i))) {
            addMessage("Track %d start (%d) leaves no space for %d more track(s)",
                    i+1, mUserStart[i], (int) numRegions - i - 1);
            region.offset = mMaxSizeY - (numRegions - i);
        } else {
            region.offset = mUserStart[i];
        }

        if (i >= mUserEnd.size()) {
            region.size = 1;
        } else if (mUserEnd[i] < 0) {
            addMessage("Track %d end (%d) less than 0",
                    i+1, mUserEnd[i]);
            region.size = 1;
        } else if (mUserEnd[i] < mUserStart[i]) {
            addMessage("Track %d end (%d) less than start (%d)",
                    i+1, mUserEnd[i], mUserStart[i]);
            region.size = 1;
        } else if (mUserEnd[i] < (int) region.offset) {
            // Start has been adjusted, so may be consequential
            region.size = 1;
        } else if (mUserEnd[i] > (int) mMaxSizeY - 1) {
            addMessage("Track %d end (%d) beyond last row (%d)",
                    i+1, mUserEnd[i], (int) mMaxSizeY - 1);
            region.size = mMaxSizeY - region.offset - (numRegions - i - 1);
        } else if (mUserEnd[i] > (int)(mMaxSizeY - (numRegions - i))) {
            addMessage("Track %d end (%d) leaves no space for %d more track(s)",
                    i+1, mUserEnd[i], (int) numRegions - i - 1);
            region.size = mMaxSizeY - region.offset - (numRegions - i - 1);
        } else {
            region.size = mUserEnd[i] + 1 - region.offset;
        }

        if (i >= mUserBin.size()) {
            region.binning = region.size; // Fully binned
        } else if (mUserBin[i] < 1) {
            addMessage("Track %d binning (%d) is less than 1",
                    i, mUserBin[i]);
            region.binning = 1;
        } else if (mUserBin[i] > (int) region.size) {
            addMessage("Track %d binning (%d) is less than track size (%zd)",
                    i, mUserBin[i], region.size);
            region.binning = region.size;
        } else {
            region.binning = mUserBin[i];
            if (region.size % mUserBin[i] != 0) {
                addMessage("Track %d binning (%d) does not divide size (%zd)",
                        i, mUserBin[i], region.size);
                region.size = (region.size / region.binning) * region.binning;
            }
        }

        TRACE("region[%u] offset=%zd size=%zd binning=%d",
                i, region.offset, region.size, region.binning);
    }
    /** Save the valid regions */
    mValid = regions;
}

void CCDMultiTrack::storeTrackAttributes(NDAttributeList* pAttributeList)
{
    if (pAttributeList) {
        char name[20];
        char desc[30];
        for (size_t i = 0; i < mValid.size(); i++) {
            int num = (int) i + 1;
            // Add new attributes listing.
            epicsSnprintf(name, sizeof(name), "ROI%dstart", num);
            epicsSnprintf(desc, sizeof(desc), "Track %d start", num);
            int start = mValid[i].offset;
            pAttributeList->add(name, desc, NDAttrInt32, &start);
            epicsSnprintf(name, sizeof(name), "ROI%dend", num);
            epicsSnprintf(desc, sizeof(desc), "Track %d end", num);
            int end = start + mValid[i].size - 1;
            pAttributeList->add(name, desc, NDAttrInt32, &end);
            epicsSnprintf(name, sizeof(name), "ROI%dbin", num);
            epicsSnprintf(desc, sizeof(desc), "Track %d binning", num);
            int bin = mValid[i].binning;
            pAttributeList->add(name, desc, NDAttrInt32, &bin);
        }
    }
}

size_t CCDMultiTrack::dataHeight(size_t trackNum) const
{
    return (trackNum < mValid.size())
        ? mValid[trackNum].size / mValid[trackNum].binning
        : 0;
}

size_t CCDMultiTrack::totalDataHeight() const
{
    int totalHeight = 0;
    for (size_t i = 0; i < mValid.size(); i++) {
        totalHeight += CCDMultiTrack::dataHeight(i);
    }
    return totalHeight;
}
