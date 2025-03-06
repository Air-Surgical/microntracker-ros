// Copyright 2025 Air Surgical, Inc.

#ifndef MICRONTRACKER_COMPONENTS__MTC_WRAPPER_HPP_
#define MICRONTRACKER_COMPONENTS__MTC_WRAPPER_HPP_

#include <cinttypes>  // Include standard library headers
#include <string>

namespace mtc
{
  #include "microntracker/MTC.h"
}  // namespace mtc

namespace mtr
{
mtc::mtFrameType stringToFrameType(const std::string & frameType)
{
  if (frameType == "None") {return mtc::mtFrameType::None;}
  if (frameType == "Full") {return mtc::mtFrameType::Full;}
  if (frameType == "ROIs") {return mtc::mtFrameType::ROIs;}
  if (frameType == "Alternating") {return mtc::mtFrameType::Alternating;}
  throw std::invalid_argument("Invalid frame type");
}

mtc::mtDecimation stringToDecimation(const std::string & decimation)
{
  if (decimation == "None") {return mtc::mtDecimation::None;}
  if (decimation == "Dec11") {return mtc::mtDecimation::Dec11;}
  if (decimation == "Dec21") {return mtc::mtDecimation::Dec21;}
  if (decimation == "Dec41") {return mtc::mtDecimation::Dec41;}
  throw std::invalid_argument("Invalid decimation");
}

mtc::mtBitDepth stringToBitDepth(const std::string & bitDepth)
{
  if (bitDepth == "None") {return mtc::mtBitDepth::None;}
  if (bitDepth == "Bpp14") {return mtc::mtBitDepth::Bpp14;}
  if (bitDepth == "Bpp12") {return mtc::mtBitDepth::Bpp12;}
  throw std::invalid_argument("Invalid bit depth");
}
}  // namespace mtr

#endif  // MICRONTRACKER_COMPONENTS__MTC_WRAPPER_HPP_
