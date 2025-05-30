/**********************************************************************
* Code: Control/Mathematical Tools typically used in power electronics control
* Programing Language: Embedded C
* Author: Veer Karan Goyal
**********************************************************************/
#include <RailwaysVFD_F280033.h>                        // Main include file
#include <math.h>

/*
float32 PIController1(float32 sampleTime, float32 error, float32 Kp, float32 Ki, float32 UpperLimit, float32 LowerLimit, Uint16 resettable, float32 UpperLimitforReset, float32 resetValue)
{
    static float32 errPreviousValue = 0.0;
    static float32 Ts = 0.0;
    static float32 PIControllerOutput = 0.0;

    Ts = 0.000001*sampleTime;    // micro-seconds to seconds conversion
    PIControllerOutput = PIControllerOutput + error*(Kp+Ki*(Ts/2)) + errPreviousValue*(0.0-Kp+Ki*(Ts/2));


    if (PIControllerOutput > UpperLimit)
    {
        PIControllerOutput = UpperLimit;
    }
    else if (PIControllerOutput < LowerLimit)
    {
        PIControllerOutput = LowerLimit;
    }

    if (resettable == 1)
    {
        if (PIControllerOutput >= UpperLimitforReset)
        {
            PIControllerOutput = resetValue;
        }
    }

    errPreviousValue = error;

    return PIControllerOutput;
}

float32 PIController2(float32 sampleTime, float32 error, float32 Kp, float32 Ki, float32 UpperLimit, float32 LowerLimit, Uint16 resettable, float32 UpperLimitforReset, float32 resetValue)
{
    static float32 errPreviousValue = 0.0;
    static float32 Ts = 0.0;
    static float32 PIControllerOutput = 0.0;

    Ts = 0.000001*sampleTime;    // micro-seconds to seconds conversion
    PIControllerOutput = PIControllerOutput + error*(Kp+Ki*(Ts/2)) + errPreviousValue*(0.0-Kp+Ki*(Ts/2));


    if (PIControllerOutput > UpperLimit)
    {
        PIControllerOutput = UpperLimit;
    }
    else if (PIControllerOutput < LowerLimit)
    {
        PIControllerOutput = LowerLimit;
    }

    if (resettable == 1)
    {
        if (PIControllerOutput >= UpperLimitforReset)
        {
            PIControllerOutput = resetValue;
        }
    }

    errPreviousValue = error;

    return PIControllerOutput;
}

float32 fourQuadtanInverse(float32 y, float32 x)
{
    static float32 phi = 0.0;
    // applying four quadrant tan inverse: Start
    phi = atan(y/x);

    if (y <= 0 && x >= 0)     // fourth quadrant -> no change
    {
      phi = phi;
    }
    else if (y >= 0 && x >= 0)// first quadrant -> no change
    {
      phi = phi;
    }
    else if ( y >= 0 && x <= 0)// second quadrant -> add pi to get positive phi value
    {
      phi = phi + 3.1416;
    }
    else                          // third quadrant -> subtract pi to get negative phi value
    {
      phi = phi - 3.1416;
    }

    return phi;
    // applying four quadrant tan inverse: End
}

void HarmonicOscillator(float32 omega, float32 *sine, float32 *cosine)
{
    static float32 theta = 0.0;
    static float32 omegap = 0.0;

    theta = theta + ((Ts/2)*(omega + omegap));
    if (theta >= 6.2832)
    {
      theta = 0;
    }

    omegap = omega;
    *sine = sin(theta);
    *cosine = cos(theta);
}

   // single phase PLL code
   // List of inputs required:
   // input signal, sampleTime (minimum = 12.5 us, maximum = 1500 us).
   // list of outputs:
   // Amplitude, frequency, sinTheta (phase synchronized with fundamental component of input signal), cosTheta (90 deg leading to fundamental component of input signal)
   // Minimum frequency of the input signal: 41 Hz, Maximum frequency of the input signal: 66 Hz

Uint16 singlePhasePll_ipPhaseA(float32 sampleTime, float32 ipSignal, float32 *Amplitude, float32 *frequency, float32 *sinTheta, float32 *cosTheta)
{
    static float32 Kp_pll=100.0;
    static float32 Ki_pll=2000.0;
    static float32 omegaMax = 418.4;   // 66.6 Hz X 2 * pi
    static float32 omegaMin = 250.0;   // 40 Hz
    static float32 freq = 50.0;
    static float32 omega = 314.16;
    static float32 cycleTime = 0.02;
    static float32 sampleDelayFloat = 500.0;
    static float32 QuarterSampleDelayFloat = 125.0;
    static Uint16 QuarterSampleDelayInt = 125;
    static Uint16 sampleDelayInt = 500;
    static float32 ipSignalBuffer[500];
    static Uint16 index = 0;
    static int16 n1 = 0;
    static int16 n2 = 0;
    static Uint16 firstSamplesCycleComplete = 0;
    static float32 fq = 0.0;
    static float32 fd = 0.0;
    static float32 fqe = 0.0;
    static float32 fde = 0.0;
    static float32 sine = 0.0;
    static float32 cosine = 1.0;
    static float32 phi = 0.0;
    static float32 Ts = 0.0;

    if (sampleTime < 12.5 || sampleTime > 1500.0)
    {
        *Amplitude = 0.0;
        *frequency = 0.0;
        *sinTheta = 0.0;
        *cosTheta = 0.0;
        return 0;
    }
    else
    {

        Ts = 0.000001*sampleTime;    // micro-seconds to seconds conversion
        // Calculating Number of Samples to be delayed: Start
        // computing cycle time from angular frequency: START
        freq = omega/6.2832;
        cycleTime = 1.0/freq;

        // computing cycle time from angular frequency: END
        // Computing samples equivalent cycleTime to cycleTime/4 delay: START, Inputs- cycleTime, Ts (sampling time)
        sampleDelayFloat = cycleTime/(Ts);
        QuarterSampleDelayFloat = sampleDelayFloat/4.0;

        // Computing samples equivalent to T/4 delay: END, Outputs- sampleDelayFloat
        // rounding of sampleDelayFloat and QuarterSampleDelayFloat to nearest integer: START, Input- sampleDelayFloat, QuarterSampleDelayFloat
        QuarterSampleDelayInt   = QuarterSampleDelayFloat;
        QuarterSampleDelayFloat = QuarterSampleDelayFloat - QuarterSampleDelayInt;
        if (QuarterSampleDelayFloat > 0.5)
        {
          QuarterSampleDelayInt = QuarterSampleDelayInt + 1;
        }

        sampleDelayInt   = sampleDelayFloat;
        sampleDelayFloat = sampleDelayFloat - sampleDelayInt;
        if (sampleDelayFloat > 0.5)
        {
          sampleDelayInt = sampleDelayInt + 1;
        }
        // rounding of sampleDelayFloat to nearest integer: END, Output- sampleDelayInt, QuarterSampleDelayInt
        // Calculating Number of Samples to be delayed: End

        // Preparing buffer for input signal T/4 delay: Start
        ipSignalBuffer[index] = ipSignal;
        index= index+1;
        if (index == 500)
        {
          index = 0;
        }

        n1 = index-QuarterSampleDelayInt;
        n2 = index + (500 - QuarterSampleDelayInt);

        if ((index == 499) && (firstSamplesCycleComplete == 0))
        {
          firstSamplesCycleComplete = 1;
        }
        // Preparing buffer for input signal T/4 delay: End

        // Single Phase to Two Phase Transformation using T/4 Delay: Start
        if (firstSamplesCycleComplete==0)
        {
          fq = 0;
        }
        else if (index >= QuarterSampleDelayInt)
        {
          fq = ipSignalBuffer[n1];
        }
        else if (index < QuarterSampleDelayInt)
        {
          fq = ipSignalBuffer[n2];
        }
        // Single Phase to Two Phase Transformation using T/4 Delay: End


        // Feeding input signal to d axis: Start
        fd = ipSignal;
        // Feeding input signal to d axis: End


        // Transformation to positively rotating reference frame: Start
        fqe = fd*cosine + fq*sine;
        fde = fd*sine   - fq*cosine;
        // Transformation to positively rotating reference frame: End
        // Note: This transformation aligns input signal with sine output of PLL

        // applying four quadrant tan inverse: Start
        phi = fourQuadtanInverse(fqe, fde);
        // applying four quadrant tan inverse: End

        // PI Controller + VCO if quarter cycle is complete
        if (firstSamplesCycleComplete==1)
        {
          HarmonicOscillator(omega, &sine, &cosine);
          omega = PIController1(sampleTime, phi, Kp_pll, Ki_pll, omegaMax, omegaMin, 0, 1000.0, 0);

        }

        *Amplitude = fde;
        *frequency = freq;
        *sinTheta = sine;
        *cosTheta = cosine;
        return 1;
    }
}

float32 PRController1(float32 sampleTime, float32 omega,float32 error, float32 Kp, float32 Kr, float32 UpperLimit, float32 LowerLimit)
{
    static float32 errPreviousValue = 0.0;
    static float32 errPreviousToPreviousValue = 0.0;
    static float32 Ts = 0.0;
    static float32 PRControllerOutput = 0.0;
    static float32 x = 0.0;
    static float32 pr_p = 0.0;
    static float32 pr_r = 0.0;
    static float32 pr_rp = 0.0;
    static float32 pr_rpp = 0.0;

    Ts = 0.000001*sampleTime;    // micro-seconds to seconds conversion
    x = (omega*omega)*(Ts*Ts);

    pr_p = Kp*error;
    pr_r = ((pr_rp*(8.0-(2.0*x)))-(pr_rpp*(4.0+x))+(4.0*Kr*Ts)*(error-errPreviousToPreviousValue))/(4.0+x);

    PRControllerOutput = pr_p + pr_r;

    if (PRControllerOutput > UpperLimit)
    {
        PRControllerOutput = UpperLimit;
    }
    else if (PRControllerOutput < LowerLimit)
    {
        PRControllerOutput = LowerLimit;
    }

    errPreviousToPreviousValue = errPreviousValue;
    errPreviousValue = error;
    pr_rpp = pr_rp;
    pr_rp = pr_r;

    return PRControllerOutput;
}

float32 PRController2(float32 sampleTime, float32 omega,float32 error, float32 Kp, float32 Kr, float32 UpperLimit, float32 LowerLimit)
{
    static float32 errPreviousValue = 0.0;
    static float32 errPreviousToPreviousValue = 0.0;
    static float32 Ts = 0.0;
    static float32 PRControllerOutput = 0.0;
    static float32 x = 0.0;
    static float32 pr_p = 0.0;
    static float32 pr_r = 0.0;
    static float32 pr_rp = 0.0;
    static float32 pr_rpp = 0.0;

    Ts = 0.000001*sampleTime;    // micro-seconds to seconds conversion
    x = (omega*omega)*(Ts*Ts);

    pr_p = Kp*error;
    pr_r = ((pr_rp*(8.0-(2.0*x)))-(pr_rpp*(4.0+x))+(4.0*Kr*Ts)*(error-errPreviousToPreviousValue))/(4.0+x);

    PRControllerOutput = pr_p + pr_r;

    if (PRControllerOutput > UpperLimit)
    {
        PRControllerOutput = UpperLimit;
    }
    else if (PRControllerOutput < LowerLimit)
    {
        PRControllerOutput = LowerLimit;
    }

    errPreviousToPreviousValue = errPreviousValue;
    errPreviousValue = error;
    pr_rpp = pr_rp;
    pr_rp = pr_r;

    return PRControllerOutput;
}

void abcToqdTransformation(float32 fa, float32 fb, float32 fc, float32 *fq, float32 *fd)
{
    *fq = fa;
    *fd = (1.0/sqrt(3.0))*(fc - fb);
}

void stationaryToPositiveRotatingTransformation(float32 fq, float32 fd, float32 sine, float32 cosine, float32 *fqePve, float32 *fdePve)
{
    // Transformation to positively rotating reference frame
    *fqePve = fq*cosine - fd*sine;
    *fdePve = fd*cosine + fq*sine;
}

void stationaryToNegativeRotatingTransformation(float32 fq, float32 fd, float32 sine, float32 cosine, float32 *fqeNve, float32 *fdeNve)
{
    // Transformation to negatively rotating reference frame
    *fqeNve = fq*cosine + fd*sine;
    *fdeNve = fd*cosine - fq*sine;
}

void DDSRFHarmonicOscillator(float32 omega, float32 *sine, float32 *cosine, float32 *sine2, float32 *cosine2)
{
    static float32 theta = 0.0;
    static float32 omegap = 0.0;

    theta = theta + ((Ts/2)*(omega + omegap));
    if (theta >= 6.2832)
    {
      theta = 0;
    }

    omegap = omega;
    *sine = sin(theta);
    *cosine = cos(theta);
    *sine2 = sin(2.0*theta);
    *cosine2 = cos(2.0*theta);
}

float32 LPF1(float32 sampleTime, float32 ipSignal, float32 wf)
{
    static float32 opSignal = 0.0;
    static float32 ipSignalPrev = 0.0;
    static float32 Ts = 0.0;
    static float32 af = 0.0;

    Ts = 0.000001*sampleTime;
    af = Ts*wf;

    opSignal = ((opSignal*(1-(af/2)))+((af/2)*(ipSignalPrev+ipSignal)))/(1+(af/2));
    ipSignalPrev = ipSignal;

    return opSignal;

}

float32 LPF2(float32 sampleTime, float32 ipSignal, float32 wf)
{
    static float32 opSignal = 0.0;
    static float32 ipSignalPrev = 0.0;
    static float32 Ts = 0.0;
    static float32 af = 0.0;

    Ts = 0.000001*sampleTime;
    af = Ts*wf;

    opSignal = ((opSignal*(1-(af/2)))+((af/2)*(ipSignalPrev+ipSignal)))/(1+(af/2));
    ipSignalPrev = ipSignal;

    return opSignal;

}


float32 LPF3(float32 sampleTime, float32 ipSignal, float32 wf)
{
    static float32 opSignal = 0.0;
    static float32 ipSignalPrev = 0.0;
    static float32 Ts = 0.0;
    static float32 af = 0.0;

    Ts = 0.000001*sampleTime;
    af = Ts*wf;

    opSignal = ((opSignal*(1-(af/2)))+((af/2)*(ipSignalPrev+ipSignal)))/(1+(af/2));
    ipSignalPrev = ipSignal;

    return opSignal;

}


float32 LPF4(float32 sampleTime, float32 ipSignal, float32 wf)
{
    static float32 opSignal = 0.0;
    static float32 ipSignalPrev = 0.0;
    static float32 Ts = 0.0;
    static float32 af = 0.0;

    Ts = 0.000001*sampleTime;
    af = Ts*wf;

    opSignal = ((opSignal*(1-(af/2)))+((af/2)*(ipSignalPrev+ipSignal)))/(1+(af/2));
    ipSignalPrev = ipSignal;

    return opSignal;

}


// sampleTime is in micro-second
Uint16 DDSRFthreePhasePll(float32 sampleTime, float32 ipSignalA, float32 ipSignalB,  float32 ipSignalC, float32 *fqePve, float32 *fdePve, float32 *fqeNve, float32 *fdeNve, float32 *Omega, float32 *sinTheta, float32 *cosTheta)
{
    static float32 omegaMax = 418.4;   // 66.6 Hz X 2 * pi
    static float32 omegaMin = 250.0;   // 40 Hz
    static float32 omega = 314.16;
    static float32 sine = 0.0;
    static float32 cosine = 1.0;
    static float32 phi = 0.0;
    //static float32 freq = 0.0;

    // Variables related to DD-SRF PLL


    static float32 Kp = 100.0;
    static float32 Ki = 2000.0;
    static float32 vq_meas = 0;
    static float32 vd_meas = 0;
    static float32 vqe = 0;
    static float32 vde = 0;
    static float32 vqe_n = 0;
    static float32 vde_n = 0;
    static float32 vqe_f = 0;
    static float32 vde_f = 0;
    static float32 vqe_nf = 0;
    static float32 vde_nf = 0;
    static float32 vqedc = 0;
    static float32 vdedc = 0;
    static float32 vqe_ndc = 0;
    static float32 vde_ndc = 0;
    static float32 sine2 = 0;
    static float32 cosine2 = 1;



    if (sampleTime < 12.5 || sampleTime > 1500.0)
    {
        *fqePve = 0.0;
        *fdePve = 0.0;
        *fqeNve = 0.0;
        *fdeNve = 0.0;
        *Omega = 0.0;
        *sinTheta = 0.0;
        *cosTheta = 0.0;
        return 0;
    }
    else
    {


        //Ts = 0.000001*sampleTime;    // micro-seconds to seconds conversion


        // *********************************************************************
        // DD-SRF PLL Code : START, inputs - ipSignalA, ipSignalB, ipSignalC
        // *********************************************************************
        //  omega = 314.16;
        // abc-qd transformation (three phase to stationary two phase transformation)
        // Assumption: Three phase three wire system
        abcToqdTransformation(ipSignalA, ipSignalB, ipSignalC, &vq_meas, &vd_meas);

        // VCO Code
        DDSRFHarmonicOscillator(omega, &sine, &cosine, &sine2, &cosine2);

        // Transformation to positively rotating reference frame
        stationaryToPositiveRotatingTransformation(vq_meas, vd_meas, sine, cosine, &vqe, &vde);

        // Transformation to negatively rotating reference frame
        stationaryToNegativeRotatingTransformation(vq_meas, vd_meas, sine, cosine, &vqe_n, &vde_n);

        // Passing the decoupled quantities through LPF (Sample Time is 40 us, cut off frequency is 555 Hz)
        vqe_f = LPF1(sampleTime, vqedc, 555.0);
        vde_f = LPF2(sampleTime, vdedc, 555.0);
        vqe_nf = LPF3(sampleTime, vqe_ndc, 555.0);
        vde_nf = LPF4(sampleTime, vde_ndc, 555.0);


        // Canceling out 100 Hz components in both the reference frames
        vqedc = vqe - vqe_nf*cosine2 + vde_nf*sine2;
        vdedc = vde - vde_nf*cosine2  - vqe_nf*sine2;
        vqe_ndc = vqe_n - vqe_f*cosine2 - vde_f*sine2;
        vde_ndc = vde_n - vde_f*cosine2 + vqe_f*sine2;

        // applying four quadrant tan inverse: Start
        phi = fourQuadtanInverse(vqedc, vdedc);

        // applying four quadrant tan inverse: End

        omega = PIController1(sampleTime, phi, Kp, Ki, omegaMax, omegaMin, 0, 1000.0, 0);
        //freq = omega/6.2832;

        // **********************************************************************
        // DD-SRF PLL Code : END, outputs : omega, vqe_f, vde_f, vqe_nf, vde_nf
        // **********************************************************************
        *fqePve = vqe;
        *fdePve = vde;
        *fqeNve = vqe_nf;
        *fdeNve = vde_nf;
        *Omega = omega;
        *sinTheta = sine;
        *cosTheta = cosine;

        return 1;
    }
}

Uint16 phaseSequenceDetector(float32 f12, float32 f23)
{
    static Uint16 SequencingComplete = 0;
    static Uint16 PositiveZeroCross = 0;
    static Uint16 NegativeZeroCross = 0;
    static float32 f12Prev = 0.0;
    static float32 f23Prev = 0.0;
    static Uint16 OneIsA_TwoisB = 1;
    static Uint16 OneIsC_TwoisB = 2;
    static Uint16 returnValue = 0;
    static Uint16 b1 = 0;
    static Uint16 c1 = 0;

    if (SequencingComplete == 0)
    {
       if (PositiveZeroCross == 0)
       {
           if (f12Prev <= 0 && f12 > 0)
           {
               PositiveZeroCross = 1;
           }
       }
       else if (PositiveZeroCross == 1 && NegativeZeroCross == 0)
       {
           if (f23Prev >= 0 && f23 < 0)
           {
               NegativeZeroCross = 1;
               c1 = c1 + 1;
               b1 = b1;
               PositiveZeroCross = 0;
               NegativeZeroCross = 0;
               if (c1 == 50)
               {
                 SequencingComplete = 1;
               }
           }
           else if (f23Prev <= 0 && f23 > 0)
           {
               NegativeZeroCross = 1;
               c1 = c1;
               b1 = b1 + 1;
               PositiveZeroCross = 0;
               NegativeZeroCross = 0;
               if (b1 == 50)
               {
                 SequencingComplete = 1;
               }
           }

       }
       returnValue = 0;

    }

    else
    {
        if (c1 == 50 && b1 < 50)
        {
            returnValue = OneIsC_TwoisB;

        }

        else if (b1 == 50 && c1 < 50)
        {
            returnValue =  OneIsA_TwoisB;
        }

    }

    // storing previous samples
    f12Prev = f12;
    f23Prev = f23;

    return returnValue;
}
*/
void SineTriangleToSVPWM(float fa, float fb, float fc, float *faSVPWM, float *fbSVPWM, float *fcSVPWM)
{
    static float Max = 0.0;
    static float Min = 0.0;
    static float commonMode = 0.0;

    //--------------------------- Finding Maximum Value ----------------------------

    if ((fa>=fb) && (fa>=fc))
    {
        Max = fa;
    }
    else if ((fb>=fa) && (fb>=fc))
    {
        Max = fb;
    }
    else
    {
        Max = fc;
    }

   //--------------------------- Finding Minimum Value ----------------------------

    if ((fa<=fb) && (fa<=fc))
    {
        Min = fa;
    }
    else if ((fb<=fa) && (fb<=fc))
    {
        Min = fb;
    }
    else
    {
        Min = fc;
    }

   //--------------- Generation of A, B, C Phase Space Vector Modulating Signals  -----------------------

    commonMode = -0.5*(Min + Max);     // Common mode voltage

    //commonMode  = 0;
    *faSVPWM = fa + commonMode;                // A-phase conventional space vector modulating signal
    *fbSVPWM = fb + commonMode;                // B-phase conventional space vector modulating signal
    *fcSVPWM = fc + commonMode;                // C-phase conventional space vector modulating signal
}
/*
void stationary_qdToabcTransformation(float32 fq, float32 fd, float32 *fa, float32 *fb, float32 *fc)
{
    *fa = fq;
    *fb = (-1.0*((fq/2.0) + (sqrt(3.0)*(fd/2.0))));
    *fc = (((sqrt(3.0)*fd)/2.0) - (fq/2.0));
}

void positiveRotating_qd_To_stationary_qdTransformation(float32 fqe, float32 fde, float32 sine, float32 cosine, float32 *fq, float32 *fd)
{
    *fq = fqe*cosine + fde*sine;
    *fd = fde*cosine - fqe*sine;
}

void negativeRotating_qd_To_stationary_qdTransformation(float32 fqeNve, float32 fdeNve, float32 sine, float32 cosine, float32 *fq, float32 *fd)
{
    *fq = fqeNve*cosine - fdeNve*sine;
    *fd = fdeNve*cosine + fqeNve*sine;
}

void DDSRFConstantDcVoltageRefCurrentCalculator(float32 Pref, float32 vqe, float32 vde, float32 vqeNve, float32 vdeNve, float32 *iqeRef, float32 *ideRef, float32 *iqeRefNve, float32 *ideRefNve)
{
   static float32 D = 0.0;

   D = ((vde*vde) + (vqe*vqe)) - ((vdeNve*vdeNve) + (vqeNve*vqeNve));
   *iqeRef = (2.0*Pref*vqe)/(3.0*D);
   *ideRef = (2.0*Pref*vde)/(3.0*D);
   *iqeRefNve = (-2.0*Pref*vqeNve)/(3.0*D);
   *ideRefNve = (-2.0*Pref*vdeNve)/(3.0*D);
}

float32 VoltageController(float32 sampleTime, float32 error, float32 Kp, float32 Ki, float32 UpperLimit, float32 LowerLimit, Uint16 resettable, float32 UpperLimitforReset, float32 resetValue)
{
    static float32 errPreviousValue = 0.0;
    static float32 Ts = 0.0;
    static float32 PIControllerOutput = 0.0;

    Ts = 0.000001*sampleTime;    // micro-seconds to seconds conversion
    PIControllerOutput = PIControllerOutput + error*(Kp+Ki*(Ts/2)) + errPreviousValue*(0.0-Kp+Ki*(Ts/2));


    if (PIControllerOutput > UpperLimit)
    {
        PIControllerOutput = UpperLimit;
    }
    else if (PIControllerOutput < LowerLimit)
    {
        PIControllerOutput = LowerLimit;
    }

    if (resettable == 1)
    {
        if (PIControllerOutput >= UpperLimitforReset)
        {
            PIControllerOutput = resetValue;
        }
    }

    errPreviousValue = error;

    return PIControllerOutput;
}

*/
/*
float ntcResistanceToTemperature(float Rntc, float R25, float Bvalue, float Room_Temperature )
    {
        powerDeviceTemperature = ((1.0 / (logf(Rntc / R25) * (1.0 / Bvalue))) + (1.0 / Room_Temperature)) - 273.15;

        return powerDeviceTemperature;
    }
*/
//float ntcResistanceToTemperature(float Rntc, float R25, float Bvalue, float Room_Temperature )
//    {
//        powerDeviceTemperature = (1.0 / ((logf(Rntc / R25) * (1.0 / Bvalue)) + (1.0 / Room_Temperature))) - 273.15;
//
//        return powerDeviceTemperature;
//    }
//
float voltsInvToModulationIndexSVPWM(float dcLinkVoltage, float lineTolineRmsVoltage){
    static float voltsPhaseNeutralRms = 0.0;
    static float voltsPhaseNeutralPeak = 0.0;
    static float modulationIndex = 0.0;
    voltsPhaseNeutralRms = lineTolineRmsVoltage/1.732;
    voltsPhaseNeutralPeak = voltsPhaseNeutralRms*1.414;
    modulationIndex = (2.0*voltsPhaseNeutralPeak)/dcLinkVoltage;
    if (modulationIndex > 1.12){
        modulationIndex = 1.12;
    }

    return modulationIndex;
}
