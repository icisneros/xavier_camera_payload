#! /usr/bin/python

# Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# This script tracks memory carvouts for T186. It will print a human readable
# table to stderr and C defines to stdout. The recomended usage is:
#	./ipc_alloc.py > ipc_alloc.h
#
# ****************************************************************************
# If you modify any existing carvout, you need to update http://nvbugs/1662067
# ****************************************************************************

from __future__ import print_function

import sys

class memory_range:
	# The start addess is inclusive and the end address is exclusive
	def __init__(self, name, start, end, granule=4096):
		self.name = name
		self.start = start
		self.end = end
		self.granule=granule
		self.reservations = []

	def reserve(self, r):
		assert(all([not x.intersect(r.start, r.end)
			    for x in self.reservations]))
		self.reservations += [r]

	# Find the first free reservation
	def find_reservation(self, size, debug=False):
		for start in range(self.start, self.end, self.granule):
			end = start + size
			if all([not x.intersect(start, end)
				for x in self.reservations]):
				return [start, end]
		return None
	def __repr__(self):
		return "\n".join(  ["%8s [0x%08x, 0x%08x)" % (self.name,
							      self.start,
							      self.end)]
				 + ["  %s" % x for x in self.reservations])

class carveout:
	def __init__(self, name, memory_range, start, end):
		self.name = name
		self.memory_range = memory_range
		self.start = start
		self.end = end
		memory_range.reserve(self)

	# Return true is [start, end) intersects with this
	def intersect(self, start, end):
		if start <= self.start and end >= self.start:
			return True
		if start >= self.start and start < self.end:
			return True
		else:
			return False

	def __repr__(self):
		return "%24s 0x%08x [0x%08x, 0x%08x)" % (self.name,
							 self.end - self.start,
							 self.start,
							 self.end)
	def define(self):
		s = "\n".join(["#define %s_START 0x%x" % (self.name,
							  self.start),
			       "#define %s_LEN 0x%x" % (self.name,
							self.end - self.start)])
		return s

# Dynamically allocate a carvout in the lowest availible slot
class dynamic_carveout(carveout):
	def __init__(self, name, memory_range, size):
		r = memory_range.find_reservation(size)
		assert(r is not None)
		[start, end] = r
		carveout.__init__(self, name, memory_range, start, end)

# Helper function to make adding an IPC easier
def add_ipc(m1, m2, memory_range, size=0x1000):
	return [dynamic_carveout("%s_TO_%s" % (m1, m2), memory_range, size),
		dynamic_carveout("%s_TO_%s" % (m2, m1), memory_range, size)]


# ---- Memory region definitions ----
ram = memory_range("DRAM", 0x80000000, 0xc0000000)
sys_ram = memory_range("SysRAM", 0x30000000, 0x30060000)
ranges = [ram, sys_ram]

# ---- Carvouts ----
carveout("TZRAM", sys_ram, 0x30000000, 0x30040000)
dynamic_carveout("MCE", sys_ram, 0x1000)
dynamic_carveout("RESERVED", sys_ram, 0x7000)
# ---- IPC carvouts ----
ipcs = []
ipcs += add_ipc("BPMP", "MCE", sys_ram)
ipcs += add_ipc("BPMP", "SPE", sys_ram)
ipcs += add_ipc("CPU_SECURE", "BPMP", sys_ram)
ipcs += add_ipc("CPU_NON_SECURE", "BPMP", sys_ram)

# Print a human readable table to std err
for r in ranges:
	print(r, file=sys.stderr)

# Print #define's to stdout for use in a header
for ipc in ipcs:
	print(ipc.define())
