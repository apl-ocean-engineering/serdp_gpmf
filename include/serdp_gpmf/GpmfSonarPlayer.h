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

#pragma once

#include <memory>
#include <string>

#include "gpmf-parser/GPMF_parser.h"
#include "liboculus/SonarPlayer.h"

namespace serdp_gpmf {

std::shared_ptr<liboculus::SonarPlayerBase> OpenFile(const std::string &filename);

///
///
///
class GPMFSonarPlayer : public liboculus::SonarPlayerBase {
public:
  GPMFSonarPlayer();
  GPMFSonarPlayer( GPMF_stream *stream );
  virtual ~GPMFSonarPlayer();

  virtual bool open(const std::string &filename);
  //bool setStream(GPMF_stream *stream);

  virtual bool isOpen() const { return _valid; }

  void close();

  virtual bool eof();
  virtual void rewind();

  // char *nextPacket();
  virtual bool nextPing( liboculus::SimplePingResult &ping );

  void dumpGPMF(void);

private:
  GPMF_stream *_stream;
  bool _valid;
  std::string _buffer;
};


} // namespace liboculus
