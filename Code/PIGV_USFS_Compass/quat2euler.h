/*
 * Copyright (c) 2022 Gregory Tomasch and Simon D. Levy.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

static void quat2euler(float qt[4], float angles[3])
{
    static const float RAD2DEG = 57.2957795;

    // MAXUSFS Quaternion is ENU

    float heading = RAD2DEG *
        atan2f(2.0f*(qt[1]*qt[2] - qt[0]*qt[3]),
               qt[0]*qt[0] - qt[1]*qt[1] + qt[2]*qt[2] - qt[3]*qt[3]);

    if(heading < 0.0f) heading += 360.0f;  // Convert heading to 0 - 360deg range

    angles[2] = heading;

    angles[1] = RAD2DEG * asinf(2.0f*(qt[2]*qt[3] + qt[0]*qt[1]));

    angles[0] = RAD2DEG *
        atan2f(2.0f*(qt[0]*qt[2] - qt[1]*qt[3]),
               qt[0]*qt[0] - qt[1]*qt[1] - qt[2]*qt[2] + qt[3]*qt[3]);
}
