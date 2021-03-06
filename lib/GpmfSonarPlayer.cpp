/*
 * Copyright (c) 2017-2020 Aaron Marburg <amarburg@uw.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of University of Washington nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <fstream>
#include <iostream>

#include "g3log/g3log.hpp"

#include "liboculus/DataTypes.h"
#include "liboculus/SimplePingResult.h"
#include "liboculus/SonarPlayer.h"

#include "serdp_gpmf/GpmfSonarPlayer.h"

namespace serdp_gpmf {

using namespace std;
using namespace liboculus;


/// Static function which automatically detects file type
shared_ptr<SonarPlayerBase> OpenFile(const string &filename) {
  std::ifstream f(filename);

  if (!f.is_open())
    return nullptr;

  char c;
  f.get(c);
  if (c == 0x44) {

    char d;
    f.get(d);

    if (d == 0x45) {
      LOG(DEBUG) << "I think this is an GPMF file.";
      return shared_ptr<SonarPlayerBase>(new GPMFSonarPlayer());
    }

  } else if (c == 0x53) {
    LOG(DEBUG) << "I think this is an raw sonar data.";
    return shared_ptr<SonarPlayerBase>(new RawSonarPlayer());
  }

  return nullptr;
}

//--- GPMFSonarPlayer --

GPMFSonarPlayer::GPMFSonarPlayer()
    : SonarPlayerBase(), _stream( new GPMF_stream ), _valid(false), _buffer()
{;}

GPMFSonarPlayer::GPMFSonarPlayer( const std::shared_ptr<GPMF_stream> &stream )
    : SonarPlayerBase(), _stream( stream ), _valid(true), _buffer()
{;}


GPMFSonarPlayer::~GPMFSonarPlayer() {
  if( _input ) {
    _input.close();
  }
}

bool GPMFSonarPlayer::open(const std::string &filename) {
  {
    bool retval = SonarPlayerBase::open(filename);
    if (!retval)
      return retval;
  }

  _input.seekg(0, std::ios::end);
  const size_t sz = _input.tellg();
  _buffer.resize(sz, '\0');
  _input.seekg(0, std::ios::beg);

  _input.read(&_buffer[0], sz);

  // _buffer.assign((std::istreambuf_iterator<char>(_input)),
  //             std::istreambuf_iterator<char>());

  LOG(DEBUG) << "Loading " << _buffer.size() << " bytes";

  {
    auto retval = GPMF_Init(_stream.get(), (unsigned int *)_buffer.c_str(), (_buffer.size()));

    if( retval != GPMF_OK ) {
      LOG(WARNING) << "Unable to initialize GPMF structure; err = " << retval;
      return false;
    }

  }

  {
    auto retval = GPMF_Validate(_stream.get(), GPMF_RECURSE_LEVELS);
    if( retval != GPMF_OK ) {
      LOG(WARNING) << "GPMF structure is not valid; err = " << retval;
      return false;
    }
  }

  {
    auto retval =
        GPMF_FindNext(_stream.get(), STR2FOURCC("OCUS"), GPMF_RECURSE_LEVELS);

    if (retval != GPMF_OK) {
      LOG(INFO) << "Unable to find Oculus sonar data in GPMF file (err "
                << retval << ")";
      return false;
    }
  }

  return true;
}

bool GPMFSonarPlayer::eof() {
  // How to handle this?
  auto retval = GPMF_Validate(_stream.get(), GPMF_RECURSE_LEVELS);
  return retval == GPMF_ERROR_BUFFER_END;
}

void GPMFSonarPlayer::rewind() {
  if (_valid) {
    GPMF_ResetState(_stream.get());
  }
}

void GPMFSonarPlayer::close() { _valid = false; }

void GPMFSonarPlayer::dumpGPMF() {
  auto key = GPMF_Key(_stream.get());
  LOG(INFO) << "Current key \"" << char((key >> 0) & 0xFF)
            << char((key >> 8) & 0xFF) << char((key >> 16) & 0xFF)
            << char((key >> 24) & 0xFF) << "\" (" << std::hex << key << ")";
  LOG(INFO) << "Current type " << GPMF_Type(_stream.get());
  LOG(INFO) << "Current device ID " << std::hex << GPMF_DeviceID(_stream.get());

  char deviceName[80];
  GPMF_DeviceName(_stream.get(), deviceName, 79);
  LOG(INFO) << "Current device name " << deviceName;

  LOG(INFO) << "Current struct size " << GPMF_StructSize(_stream.get());
  LOG(INFO) << "Current repeat size " << GPMF_Repeat(_stream.get());
  LOG(INFO) << "Current payload sample count " << GPMF_PayloadSampleCount(_stream.get());
  LOG(INFO) << "Current elements in struct " << GPMF_ElementsInStruct(_stream.get());
  LOG(INFO) << "Current raw data size " << GPMF_RawDataSize(_stream.get());
}

bool GPMFSonarPlayer::nextPing( SimplePingResult &ping ) {

  auto key = GPMF_Key(_stream.get());
  if (key != STR2FOURCC("OCUS"))
    return false;

  MessageHeader header( (char *)GPMF_RawData(_stream.get()), GPMF_RawDataSize(_stream.get()) );
  // char *data = (char *)GPMF_RawData(_stream);
  // CHECK(data != nullptr);

  if (!header.valid()) {
    LOG(INFO) << "Invalid header";
    return false;
  }

  auto retval = GPMF_FindNext(_stream.get(), STR2FOURCC("OCUS"), GPMF_RECURSE_LEVELS);
  if (retval != GPMF_OK) {
    _valid = false;
  }

  ping = SimplePingResult( header );
  return true;
}

} // namespace serdp_gpmf
