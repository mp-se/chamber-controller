/*
 * Chamber Controller
 * Copyright (c) 2024-2026 Magnus
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#ifndef SRC_PIDPUSH_HPP_
#define SRC_PIDPUSH_HPP_

#include <basepush.hpp>
#include <chamberreport.hpp>
#include <pidconfig.hpp>

class PidPush : public BasePush {
 private:
  PidConfig* _pidConfig;

 public:
  explicit PidPush(PidConfig* config);

  // POST the report to brewgraph_url (used as it is) and read the reply.
  // Returns false if there is nothing to report, the request failed or the
  // reply is not understood.
  bool pollChamber(const ChamberReport& report, ChamberReply& reply);
};

#endif  // SRC_PIDPUSH_HPP_

// EOF
