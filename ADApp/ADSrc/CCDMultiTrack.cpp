/**
 * Area Detector class enabling multi-ROI driver for the Andor CCD.
 * This class is used by CCD camera modules that permit multiple regions-of-interest.
 *
 * Multi-ROI is typically used for multi-track spectrocopy application.
 *
 * There are 3 use cases:
 *    1 - The user sets only the track start array.
 *        This provides a single height track at thos positions.
 *    2 - The user sets the start and end tracks arrays.
 *        This provides a fully-binned track between the start and end positions.
 *    3 - The user provides start, end and binning values.
 *        This provides (a less than fully binned) track between the start and end positions.
 *
 * @author Peter Heesterman
 * @date Nov 2019
 *
 */

#include "CCDMultiTrack.h"

/** Parameter strings to tie into asyn records */
static const char* CCDMultiTrackStartString = "CCD_MULTI_TRACK_START";
static const char* CCDMultiTrackEndString = "CCD_MULTI_TRACK_END";
static const char* CCDMultiTrackBinString = "CCD_MULTI_TRACK_BIN";

CCDMultiTrack::CCDMultiTrack(asynPortDriver* asynPortDriver)
{
    mPortDriver = asynPortDriver;
    /* Semi-sensible value for old AD instances */
    mMaxSizeY = 1024;
    /* Create parameters and get indices */
    asynPortDriver->createParam(CCDMultiTrackStartString, asynParamInt32Array, &mCCDMultiTrackStart);
    asynPortDriver->createParam(CCDMultiTrackEndString, asynParamInt32Array, &mCCDMultiTrackEnd);
    asynPortDriver->createParam(CCDMultiTrackBinString, asynParamInt32Array, &mCCDMultiTrackBin);
}

/** Set size of CCD */
void CCDMultiTrack::setMaxSize(size_t maxSizeY) {
    mMaxSizeY = maxSizeY;
    validate();
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
        puserArray = &mUserStart;
    }
    else if (function == mCCDMultiTrackEnd) {
        puserArray = &mUserEnd;
    }
    else if (function == mCCDMultiTrackBin) {
        puserArray = &mUserBin;
    }
    else {
        status = asynError;             // Not our parameter
    }

    if (puserArray) {
        if (! (valueArray == *puserArray)) {
            *puserArray = valueArray;
            validate();

            for (unsigned m = 0; m < mMessages.size(); m++) {
                asynPrint(pasynUser, ASYN_TRACE_WARNING,
                        "CCDMultiTrack: %s\n", mMessages[m].c_str());
            }
        }
    }

    return status;
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
void CCDMultiTrack::validate()
{
    std::vector<NDDimension_t> regions;

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
            region.offset = mMaxSizeY - 1;
        } else if (mUserStart[i] > (int) (mMaxSizeY - (numRegions - i))) {
            addMessage("Track %d start (%d) leaves no space for %d more track(s)",
                    i+1, mUserStart[i], (int) numRegions - i - 1);
            region.offset = mMaxSizeY - (numRegions - 1);
        } else {
            region.offset = mUserStart[i];
        }
        // FIXME size, binning
    }
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
