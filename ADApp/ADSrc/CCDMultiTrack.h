/**
 * Area Detector class enabling multi-ROI driver for the Andor CCD.
 * This class is used by CCD camera modules that permit multiple regions-of-interest.
 *
 * Multi-ROI is typically used for multi-track spectrocopy application.
 *
 * @author Peter Heesterman
 * @date Nov 2019
 *
 */
#ifndef CCD_MULTI_TRACK_H
#define CCD_MULTI_TRACK_H

#include <asynPortDriver.h>
#include <NDArray.h>
#include <ADCoreAPI.h>

class ADCORE_API CCDMultiTrack
{
 public:
    /** Constructor, supplying AD instance.
     * This allows creation of parameters and setting of validated
     * (read-back) values.
     * The optional asynUser allows output of tracing information.
     */
    CCDMultiTrack(asynPortDriver* asynPortDriver, asynUser *pasynUser=0);
    /** Set the Y size of the CCD, used for validation */
    void setMaxSize(size_t maxSizeY);
    /** Handler for int32 array setting.
     * Returns asynError if parameter unknown to CCDMultiTrack.
     */
    asynStatus writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements);
    /** Number of tracks defined (number of start positions) */
    size_t size() const {
        return mValid.size();
    }
    /** Get list of track definitions after validation */
    std::vector<NDDimension_t> validTracks() {
        return mValid;
    }
    /** Get list of messages from validation */
    std::vector<std::string> validationMessages() {
        return mMessages;
    }
    /** Elements after binning, for a given validated track */
    size_t dataHeight(size_t trackNum) const;
    /** Total elements in Y direction, after binning */
    size_t totalDataHeight() const;
    /** Set multi-track attributes on output NDArray */
    void storeTrackAttributes(NDAttributeList* pAttributeList);

    /** Legacy methods */
    int CCDMultiTrackStart() const {
        return mCCDMultiTrackStart;
    }
    int CCDMultiTrackEnd() const {
        return mCCDMultiTrackEnd;
    }
    int CCDMultiTrackBin() const {
        return mCCDMultiTrackBin;
    }
    int TrackStart(size_t trackNum) const {
        return (trackNum < size()) ? mValid[trackNum].offset : 0;
    };
    int TrackEnd(size_t trackNum) const {
        return (trackNum < size()) ? mValid[trackNum].offset + mValid[trackNum].size - 1 : 0;
    }
    int TrackHeight(size_t trackNum) const {
        return (trackNum < size()) ? mValid[trackNum].size : 1;
    }
    int TrackBin(size_t trackNum) const {
        return (trackNum < size()) ? mValid[trackNum].binning : 1;
    }
    int DataHeight() const {
        return totalDataHeight();
    }
    int DataHeight(size_t trackNum) const {
        return dataHeight(trackNum);
    }

 protected:
    /** Size of CCD in Y direction */
    size_t mMaxSizeY;
    /** Arrays as set by user */
    std::vector<int> mUserStart;
    std::vector<int> mUserEnd;
    std::vector<int> mUserBin;
    /** Regions coerced to be valid */
    std::vector<NDDimension_t> mValid;
    /** Messages from validation */
    std::vector<std::string> mMessages;

    /** Check and convert to a valid set of track definitions
     * Reads mUserStart/End/Bin, sets mValid[] and mMessages[].
     * A subclass can override this, if it needs to apply more
     * stringent constraints e.g. symmetry, binning range etc.
     */
    virtual void validate();

    /** For use within validate() */
    void addMessage(const char *fmt, ...)
        EPICS_PRINTF_STYLE(2,3);

private:
    asynPortDriver* mPortDriver;
    asynUser* mAsynUser;
    /** Parameter indexes */
    int mCCDMultiTrackStart;
    int mCCDMultiTrackEnd;
    int mCCDMultiTrackBin;
    /** Validation */
    bool mValidated;
    void _validate();
};

#endif //CCD_MULTI_TRACK_H
