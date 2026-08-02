#include "autopilot/guidance/HeadingLoopIdentifier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace arlcore::autopilot {

// A step must turn this far to be trusted as a source of saturated-region samples.
static constexpr flt64_t kLargeStepRad = 0.6981;  // 40 degrees
// ... and no further than this to be trusted as a source of unsaturated decay samples.
static constexpr flt64_t kSmallStepRad = 0.1746;  // just above 10 degrees, so a 10-degree step qualifies
// Skip the leading edge of a step so actuator lag does not enter the saturated window.
static constexpr flt64_t kSaturationSkipS = 0.5;
static constexpr flt64_t kSaturatedFraction = 0.95;
// The regression keeps well clear of the knee so curvature there cannot bias the slope.
static constexpr flt64_t kLinearRateFraction = 0.7;
static constexpr flt64_t kLinearErrFraction = 0.7;
// Below roughly compass resolution a sample carries no gain information. The floor matters:
// a probe spends most of its dwell settled at zero error, and those samples would otherwise
// swamp the regression and make a degenerate fit look well conditioned.
static constexpr flt64_t kMinRegressionErrRad = 0.0087;  // 0.5 degrees
static constexpr int32_t kMinGainSamples = 200;
static constexpr int32_t kMinOmegaMaxWindows = 3;
static constexpr flt64_t kMinGainR2 = 0.8;
static constexpr flt64_t kMaxOmegaMaxCv = 0.15;

static flt64_t median(std::vector<flt64_t>* values) {
  if (values->empty()) {
    return 0.0;
  }
  const std::size_t mid = values->size() / 2;
  std::nth_element(values->begin(), values->begin() + mid, values->end());
  return (*values)[mid];
}

//! \brief Least-squares slope through the origin, plus its coefficient of determination.
static void fitThroughOrigin(const std::vector<flt64_t>& x, const std::vector<flt64_t>& y, flt64_t* slope,
                             flt64_t* r2) {
  *slope = 0.0;
  *r2 = 0.0;
  flt64_t sxy = 0.0;
  flt64_t sxx = 0.0;
  for (std::size_t i = 0; i < x.size(); i++) {
    sxy += x[i] * y[i];
    sxx += x[i] * x[i];
  }
  if (sxx <= 0.0) {
    return;
  }
  *slope = sxy / sxx;

  flt64_t mean = 0.0;
  for (const flt64_t v : y) {
    mean += v;
  }
  mean /= static_cast<flt64_t>(y.size());
  flt64_t ssRes = 0.0;
  flt64_t ssTot = 0.0;
  for (std::size_t i = 0; i < x.size(); i++) {
    const flt64_t residual = y[i] - *slope * x[i];
    ssRes += residual * residual;
    ssTot += (y[i] - mean) * (y[i] - mean);
  }
  *r2 = ssTot > 0.0 ? 1.0 - ssRes / ssTot : 0.0;
}

//! \brief Samples grouped by step index, preserving order within each step.
static std::map<int32_t, std::vector<HeadingProbeSample>> groupBySteps(const std::vector<HeadingProbeSample>& samples) {
  std::map<int32_t, std::vector<HeadingProbeSample>> steps;
  for (const HeadingProbeSample& s : samples) {
    if (!std::isfinite(s.headingErrRad) || !std::isfinite(s.yawRateRps) || !std::isfinite(s.elapsedS)) {
      continue;
    }
    if (s.stepIndex < 0) {
      continue;
    }
    steps[s.stepIndex].push_back(s);
  }
  return steps;
}

//! \brief Median |omega| over each large step's saturated window, and the spread across them.
static void identifyOmegaMax(const std::map<int32_t, std::vector<HeadingProbeSample>>& steps, HeadingLoopModel* model) {
  std::vector<flt64_t> perWindow;
  for (const std::pair<const int32_t, std::vector<HeadingProbeSample>>& entry : steps) {
    const std::vector<HeadingProbeSample>& step = entry.second;
    if (step.empty() || std::fabs(step.front().stepRad) < kLargeStepRad) {
      continue;
    }
    const flt64_t startS = step.front().elapsedS;
    const flt64_t halfStep = 0.5 * std::fabs(step.front().stepRad);
    std::vector<flt64_t> rates;
    for (const HeadingProbeSample& s : step) {
      if (s.elapsedS - startS < kSaturationSkipS) {
        continue;
      }
      if (std::fabs(s.headingErrRad) < halfStep) {
        break;  // out of the saturated region: the loop is now tracking, not slewing
      }
      rates.push_back(std::fabs(s.yawRateRps));
    }
    if (!rates.empty()) {
      perWindow.push_back(median(&rates));
    }
  }
  model->omegaMaxWindows = static_cast<int32_t>(perWindow.size());
  if (perWindow.empty()) {
    return;
  }
  std::vector<flt64_t> copy = perWindow;
  model->omegaMaxRps = median(&copy);
  if (model->omegaMaxRps > 0.0 && perWindow.size() > 1) {
    flt64_t mean = 0.0;
    for (const flt64_t v : perWindow) {
      mean += v;
    }
    mean /= static_cast<flt64_t>(perWindow.size());
    flt64_t var = 0.0;
    for (const flt64_t v : perWindow) {
      var += (v - mean) * (v - mean);
    }
    var /= static_cast<flt64_t>(perWindow.size());
    model->omegaMaxCv = mean > 0.0 ? std::sqrt(var) / mean : 0.0;
  }
}

//! \brief Fit K over the unsaturated region, re-selecting the region once K is known.
static void identifyGain(const std::vector<HeadingProbeSample>& usable, HeadingLoopModel* model) {
  flt64_t errLimit = std::numeric_limits<flt64_t>::max();
  for (int32_t pass = 0; pass < 3; pass++) {
    std::vector<flt64_t> errs;
    std::vector<flt64_t> rates;
    std::vector<flt64_t> portErrs;
    std::vector<flt64_t> portRates;
    std::vector<flt64_t> stbdErrs;
    std::vector<flt64_t> stbdRates;
    for (const HeadingProbeSample& s : usable) {
      const bool rateOk =
          model->omegaMaxRps <= 0.0 || std::fabs(s.yawRateRps) <= kLinearRateFraction * model->omegaMaxRps;
      if (!rateOk || std::fabs(s.headingErrRad) > errLimit || std::fabs(s.headingErrRad) < kMinRegressionErrRad) {
        continue;
      }
      errs.push_back(s.headingErrRad);
      rates.push_back(s.yawRateRps);
      if (s.stepRad > 0.0) {
        stbdErrs.push_back(s.headingErrRad);
        stbdRates.push_back(s.yawRateRps);
      } else if (s.stepRad < 0.0) {
        portErrs.push_back(s.headingErrRad);
        portRates.push_back(s.yawRateRps);
      }
    }
    if (errs.empty()) {
      return;
    }
    flt64_t slope = 0.0;
    flt64_t r2 = 0.0;
    fitThroughOrigin(errs, rates, &slope, &r2);
    model->gainRpsPerRad = slope;
    model->gainR2 = r2;
    model->gainSamples = static_cast<int32_t>(errs.size());
    flt64_t unusedR2 = 0.0;
    fitThroughOrigin(portErrs, portRates, &model->gainPortRpsPerRad, &unusedR2);
    fitThroughOrigin(stbdErrs, stbdRates, &model->gainStbdRpsPerRad, &unusedR2);
    if (slope <= 0.0 || model->omegaMaxRps <= 0.0) {
      return;
    }
    errLimit = kLinearErrFraction * model->omegaMaxRps / slope;
  }
}

//! \brief The heading error still present at the last saturated sample of each large step.
static void identifySaturationKnee(const std::map<int32_t, std::vector<HeadingProbeSample>>& steps,
                                   HeadingLoopModel* model) {
  if (model->omegaMaxRps <= 0.0) {
    return;
  }
  std::vector<flt64_t> knees;
  for (const std::pair<const int32_t, std::vector<HeadingProbeSample>>& entry : steps) {
    if (entry.second.empty() || std::fabs(entry.second.front().stepRad) < kLargeStepRad) {
      continue;
    }
    flt64_t knee = 0.0;
    for (const HeadingProbeSample& s : entry.second) {
      if (std::fabs(s.yawRateRps) >= kSaturatedFraction * model->omegaMaxRps) {
        knee = std::fabs(s.headingErrRad);
      }
    }
    if (knee > 0.0) {
      knees.push_back(knee);
    }
  }
  if (!knees.empty()) {
    model->satErrKneeRad = median(&knees);
  }
}

//! \brief Closed-loop decay time constant from the small, non-saturating steps.
static void identifyDecayTau(const std::map<int32_t, std::vector<HeadingProbeSample>>& steps, HeadingLoopModel* model) {
  std::vector<flt64_t> taus;
  for (const std::pair<const int32_t, std::vector<HeadingProbeSample>>& entry : steps) {
    const std::vector<HeadingProbeSample>& step = entry.second;
    if (step.empty() || std::fabs(step.front().stepRad) > kSmallStepRad) {
      continue;
    }
    const flt64_t initial = std::fabs(step.front().headingErrRad);
    if (initial <= 1e-9) {
      continue;
    }
    // Regress ln|e| against time over the middle of the decay, where neither the leading
    // transient nor the noise floor dominates.
    std::vector<flt64_t> times;
    std::vector<flt64_t> logs;
    for (const HeadingProbeSample& s : step) {
      const flt64_t magnitude = std::fabs(s.headingErrRad);
      if (magnitude > 0.9 * initial || magnitude < 0.1 * initial || magnitude <= 1e-12) {
        continue;
      }
      times.push_back(s.elapsedS - step.front().elapsedS);
      logs.push_back(std::log(magnitude));
    }
    if (times.size() < 3) {
      continue;
    }
    flt64_t meanT = 0.0;
    flt64_t meanL = 0.0;
    for (std::size_t i = 0; i < times.size(); i++) {
      meanT += times[i];
      meanL += logs[i];
    }
    meanT /= static_cast<flt64_t>(times.size());
    meanL /= static_cast<flt64_t>(logs.size());
    flt64_t num = 0.0;
    flt64_t den = 0.0;
    for (std::size_t i = 0; i < times.size(); i++) {
      num += (times[i] - meanT) * (logs[i] - meanL);
      den += (times[i] - meanT) * (times[i] - meanT);
    }
    if (den <= 0.0 || num >= 0.0) {
      continue;  // not decaying, so there is no time constant to report
    }
    taus.push_back(-den / num);
  }
  if (!taus.empty()) {
    model->decayTauS = median(&taus);
  }
}

HeadingLoopModel identifyHeadingLoop(const std::vector<HeadingProbeSample>& samples) {
  HeadingLoopModel model;
  const std::map<int32_t, std::vector<HeadingProbeSample>> steps = groupBySteps(samples);
  if (steps.empty()) {
    return model;
  }

  std::vector<HeadingProbeSample> usable;
  for (const std::pair<const int32_t, std::vector<HeadingProbeSample>>& entry : steps) {
    usable.insert(usable.end(), entry.second.begin(), entry.second.end());
  }

  identifyOmegaMax(steps, &model);
  identifyGain(usable, &model);
  identifySaturationKnee(steps, &model);
  identifyDecayTau(steps, &model);

  if (model.gainRpsPerRad > 0.0) {
    model.headingLoopTauS = 1.0 / model.gainRpsPerRad;
    if (model.omegaMaxRps > 0.0) {
      model.satErrRad = model.omegaMaxRps / model.gainRpsPerRad;
    }
    model.modelConsistency = model.decayTauS * model.gainRpsPerRad;
  }
  const flt64_t gainMean = 0.5 * (std::fabs(model.gainPortRpsPerRad) + std::fabs(model.gainStbdRpsPerRad));
  if (gainMean > 0.0) {
    model.asymmetryFrac = std::fabs(model.gainPortRpsPerRad - model.gainStbdRpsPerRad) / gainMean;
  }

  if (model.omegaMaxWindows < kMinOmegaMaxWindows || model.omegaMaxCv > kMaxOmegaMaxCv) {
    model.verdict = "UNRELIABLE_OMEGA_MAX";
  } else if (model.gainSamples < kMinGainSamples || model.gainR2 < kMinGainR2 || model.gainRpsPerRad <= 0.0) {
    // Almost every sample sits against the rate limit, so there is no linear region to fit.
    // The knee estimate is the only usable read on where saturation begins.
    model.verdict = "SATURATION_DOMINATED";
  } else {
    model.verdict = "OK";
  }
  return model;
}

}  // namespace arlcore::autopilot
