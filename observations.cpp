/**
  \file
  \author Kevin P. Rauch
  \brief  Implements concrete Observation classes.

   $Id: observations.cpp,v 1.11 2007/10/03 22:08:31 rauch Exp $
*/
#include <iosys.h>
#include <stdlib.h>
#include <string.h>

#include <basictypes.h>
#include <utils.h>

#include <algorithm>
#include <limits>

#include "observations.h"
#include "zpec.h"


/** LED helper function to display a decimal number. */
void PutDispDecimal(WORD val, BOOL blank_zero )
{
  WORD w;
  
  w = (val / 1000) * 0x1000;
  if ((w == 0) && (blank_zero)) w = 0xF000;
  w += ((val / 100)% 10) * 0x0100;
  if (w == 0xF000) w = 0xFF00;
  w += ((val / 10)% 10) * 0x0010;
  if (w == 0xFF00) w = 0xFFF0;
  w += ((val )% 10) * 0x0001;
  putdisp(w);
}


/**
  Accumulates a single data frame.

  This method accumulates the current frame's lag counts into the internal
  accumulation buffer.

  \param begin Pointer to start of frame buffer.
  \param end   Pointer to (one past) end of frame buffer.
*/
void BasicObservation::processFrame(const lag_count_t *begin,
                                    const lag_count_t *end)
{
  unsigned iBuffer = nFrames_ % nBuffers_;

  // Accumulate the new frame.
  ++nFrames_;
  std::transform(lagBuffer_.begin() + nLags_*iBuffer,
                 lagBuffer_.begin() + nLags_*(iBuffer+1),
		 begin, lagBuffer_.begin() + nLags_*iBuffer,
		 std::plus<lag_count_t>());

  if (getMode() == MODE_DEVKIT) {
    // Test mode: increment LED display and log message.
    PutDispDecimal(nFrames_, true);
    WORD dips = getdipsw();
    zpec_debug("The IRQ button has been pressed %ld times"
	       " (DIP switch state is 0x%02hX)", nFrames_, dips);
  }

  lagBuffer_.setFrames(iBuffer, lagBuffer_.getFrames(iBuffer) + 1);
  lagBuffer_.setTime(iBuffer, ::TimeTick);
}


/**
  Processes a single data frame.

  This method incorporates the current lag counts into the cumulative lag
  statistics. The first several frames are used simply to derive a rough
  estimate of the means. The estimated means are used in the initial variance 
  calculation to remove the offset for enhanced numerical stability.

  The variance accumulation is split between low and high words to mitigate
  the chance of overflow.

  \param begin Pointer to start of frame buffer.
  \param end   Pointer to (one past) end of frame buffer.
*/
void StatsObservation::processFrame(const lag_count_t *begin,
                                    const lag_count_t *end)
{
  LagData::iterator pSum     = sumBuffer_.begin(),
		    eSum     = sumBuffer_.end(),
		    pSum2lsb = sum2lsbBuffer_.begin(),
		    pSum2msb = sum2msbBuffer_.begin();

  /*
     Accumulate statistics.
     WARNING: Do NOT use long long delta2! Due to unknown bug, use of a
	      long long type here corrupts the stack/heap!!!!
  */
  for (const lag_count_t *pLag = begin; pSum != eSum; ) {
    lag_count_t delta = *pLag++;
    unsigned delta2 = abs(delta);

    delta2      *= abs(delta);
    *pSum++     += delta;
    *pSum2msb++ += hi(delta2);
    *pSum2lsb++ += lo(delta2);
  }
  sumBuffer_.setFrames(0, 1+sumBuffer_.getFrames(0));
}


/**
  Computes 64-bit product x*y.
  
  \param prod Base-2^32 digits of product (prod = 2^32*prod[1] + prod[0]).
  \param x    First  multiplicand.
  \param y    Second multiplicand.
*/
void StatsObservation::mul_64bit(unsigned *prod, unsigned x, unsigned y)
{
  unsigned x1 = hi(x), x0 = lo(x), y1 = hi(y), y0 = lo(y),
	   x1y1 = x1*y1, x1y0 = x1*y0, x0y1 = x0*y1, x0y0 = x0*y0,
	   p3, p2, p1, p0;

  // Compute digits of product using elementary base-2^16 arithmetic.
  p0 = lo(x0y0);
  //
  p1 = lo(x0y1) + lo(x1y0) + hi(x0y0);
  p2 = lo(x1y1) + hi(x1y0) + hi(x0y1) + hi(p1);
  p1 = lo(p1);
  //
  p3 = hi(x1y1) + hi(p2);
  p2 = lo(p2);

  prod[1] = (p3 << 16) + p2;
  prod[0] = (p1 << 16) + p0;
}


/**
  Computes 64-bit quotient x/y.
  
  \param prod Base-2^32 digits of quotient (prod = 2^32*prod[1] + prod[0]).
              Can be the same as \a x.
  \param x    Numerator.
  \param y    Denominator.
*/
void StatsObservation::div_64bit(unsigned *quot, const unsigned *x, unsigned y)
{
  unsigned numer[4], q[4];

  // Numerator.
  numer[3] = hi(x[1]);
  numer[2] = lo(x[1]);
  numer[1] = hi(x[0]);
  numer[0] = lo(x[0]);

  // Compute digits of quotient using elementary base-2^16 arithmetic.
  unsigned carry;
  int i;
  for (carry=0, i=3; i>=0; --i) {
    unsigned part = carry + numer[i];
    q[i]  = (part / y);
    carry = (part % y) << 16;
  }

  quot[1] = (q[3] << 16) + q[2];
  quot[0] = (q[1] << 16) + q[0];
}


/**
  Computes variance using 64-bit internal precision.

  If the fixed-point scaling would cause the variance to overflow,
  it will be ignored and negative the unscaled result will be returned.
  If the unscaled result overflows, (negative) the maximum representable 
  result will be returned.

  \param scale   Fixed point scale factor (true = return/scale).
  \param sum     Sum(x) of the samples.
  \param sum2_1  Sum(x^2) of the samples (upper base-2^32 digit).
  \param sum2_0  Sum(x^2) of the samples (lower base-2^32 digit).
  \param nSamp   Number of samples.

  \return Fixed-point sample variance.
*/
int StatsObservation::computeVar(int sum, int sum2_1, int sum2_0,
                                 unsigned nSamp)
{
  unsigned asum = abs(sum), n = (nSamp>1 ? nSamp-1 : 1), sum2[2], x[2], del[2];

  // Normalize sum2 = 2^16*sum2_1 + sum2_0.
  unsigned p = lo(sum2_1) + hi(sum2_0);
  sum2[1] = hi(p) + hi(sum2_1);
  sum2[0] = (lo(p) << 16) + lo(sum2_0);

  // x = (sum*sum)/nSamp.
  mul_64bit(x, asum, asum);
  div_64bit(x, x, nSamp);

  // del = Sum(x^2) - Sum(x)^2/nSamp.
  sub_64bit(del, sum2, x);

  if (del[1] == 0) {
    // Maintain full precision: x = (scale * del) / n.
    mul_64bit(x, scaleFixed_, del[0]);
    div_64bit(x, x, n);
  } else {
    // Avoid overflow: x = scale * (del / n);
    div_64bit(x, del, n);
    if (x[1] == 0) { mul_64bit(x, scaleFixed_, x[0]); }
  }

  // Remove scaling on overflow.
  int var = x[0];
  if (intOverflow(x)) {
    div_64bit(x, del, n);
    var = (intOverflow(x) ? -std::numeric_limits<int>::max() : -(int )x[0]);
  }
  return  var;
}


/**
  Computes mean using 64-bit internal precision.

  \param sum      Sum(x) of the samples.
  \param nSamp    Number of samples.

  \return Fixed-point sample mean.
*/
int StatsObservation::computeMean(int sum, unsigned nSamp)
{
  unsigned asum = abs(sum), x[2];
  mul_64bit(x, scaleFixed_, asum);
  div_64bit(x, x, nSamp);

  int mean = (intOverflow(x) ? std::numeric_limits<int>::max() : x[0]);
  return (sum<0 ? -mean : mean);
}


/**
  Computes final lag statistics.

  This method overwrites the statistic accumulation buffers with final mean
  and variance numbers. The values are stored as fixed-point scaled integers
  carrying three decimal places of precision (the floating-point result is the
  scaled result divided by scaleFixed). Exception: if the returned variance is
  negative, it signals that no scaling has been applied to the (absolute)
  value. If the variance overflows 32-bit representation, the largest possible
  value is returned.

  \warning Further calls to processFrame() will corrupt statistics.
*/
void StatsObservation::computeStats()
{
  unsigned nSamp = sumBuffer_.getFrames(0);

  // Compute mean and variance.
  for (LagData::iterator pSum     = sumBuffer_.begin(),
			 eSum     = sumBuffer_.end(),
			 pSum2lsb = sum2lsbBuffer_.begin(),
			 pSum2msb = sum2msbBuffer_.begin();
			 pSum != eSum; ++pSum, ++pSum2lsb, ++pSum2msb) {

    // Summary statistics are processed using 64-bit internal precision.
    *pSum2lsb = computeVar(*pSum, *pSum2msb, *pSum2lsb, nSamp);
    *pSum     = computeMean(*pSum, nSamp);
  }
}


/**
  Processes a single data frame.

  This method adds samples from a single frame to the sample buffer.

  \param begin Pointer to start of frame buffer.
  \param end   Pointer to (one past) end of frame buffer.
*/
void ScopeObservation::processFrame(const lag_count_t *begin,
                                    const lag_count_t *end)
{
  // Current sample number.
  unsigned iSample = nSamples_++ / nAccum_;

  for (unsigned iChannel=0; iChannel<nChannels_; ++iChannel) {
    sampBuffer_[nChannels_*iSample + iChannel] += begin[iLag_[iChannel]];
  }
}
