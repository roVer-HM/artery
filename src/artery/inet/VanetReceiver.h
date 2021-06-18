/*
* Artery V2X Simulation Framework
* Copyright 2019-2020 Raphael Riebl
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_VANETRECEIVER_H_PNEYWKVR
#define ARTERY_VANETRECEIVER_H_PNEYWKVR

#include "inet/physicallayer/wireless/ieee80211/packetlevel/Ieee80211ScalarReceiver.h"

namespace artery
{

/**
 * VanetReceiver adds capturing of a stronger frame while already receiving a weak frame
 */
class VanetReceiver : public inet::physicallayer::Ieee80211ScalarReceiver
{
    /* TODO: necessary with new INET4 Wlan implementations? */
    /*
public:
    bool computeIsReceptionAttempted(const inet::physicallayer::IListening*, const inet::physicallayer::IReception*,
            inet::physicallayer::IRadioSignal::SignalPart, const inet::physicallayer::IInterference*) const override;

protected:
    void initialize(int stage) override;
    //const inet::physicallayer::Ieee80211ModeInd* computeReceptionIndication(const inet::physicallayer::ISnir*) const override;
    virtual const IReceptionResult *computeReceptionResult(const IListening *listening, const IReception *reception, const IInterference *interference, const ISnir *snir, const std::vector<const IReceptionDecision *> *decisions) const override;

private:
    double mCaptureThreshold;
    */
};

} // namespace artery

#endif /* ARTERY_VANETRECEIVER_H_PNEYWKVR */

