#include "FaceStatus.h"

namespace {

constexpr LedColor kOff = {0, 0, 0};
constexpr LedColor kWhiteDim = {40, 40, 40};
constexpr LedColor kGreenDim = {0, 40, 0};
constexpr LedColor kRed = {120, 0, 0};
constexpr LedColor kBlue = {0, 0, 120};

}  // namespace

StatusView statusViewFor(bool calibrating, bool gcs_linked, bool armed, bool manual_mode) {
    if (calibrating) {
        return {FaceExpression::Neutral, "キャリブレーション中", kWhiteDim};
    }
    if (!gcs_linked) {
        return {FaceExpression::Sleepy, "接続待ち", kOff};
    }
    if (!armed) {
        return manual_mode
                   ? StatusView{FaceExpression::Neutral, "マニュアル", kGreenDim}
                   : StatusView{FaceExpression::Neutral, "接続", kGreenDim};
    }
    if (manual_mode) {
        return {FaceExpression::Happy, "マニュアル操作", kBlue};
    }
    return {FaceExpression::Doubt, "アーム", kRed};
}
