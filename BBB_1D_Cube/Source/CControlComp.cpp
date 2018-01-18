/*
 * CControlComp.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: Robin Tenscher
 *      		Tim Schanz
 */

#include "../Header/CControlComp.h"

CControlComp::CControlComp()
{
	mtmp = 0;
}

CControlComp::~CControlComp()
{
}
void CControlComp::run()
{
	std::cout << "Control Comp " << std::endl;
	float f = 0.0;
	time_t time_1, time_2;

	mCBBHardware.setTorque(0.0);
	mCBBHardware.enableMotor();
	mCBBHardware.setTorque(0.0);
	while (1)
	{
		time(&time_1);	//Time measurepoint 1
		mCBBHardware.fetchValues(mADCVal, mdata1_raw, mdata2_raw);		//Collect data from sensors

		calibrate();		//Calibrate raw data
		filter();			//Filter data and calculate phi

		//Display
		float phi_disp = mstate.mPhi_C * 180.0 / 3.1415;
		std::cout << std::endl;
		std::cout << "ADC Value: " << mCalibratedData.Psi_d << std::endl;
		std::cout << "S1Phi Value: " << mCalibratedData.Phi1_d << std::endl;
		std::cout << "S1XDD Value: " << mCalibratedData.X1_dd << std::endl;
		std::cout << "S1YDD Value: " << mCalibratedData.Y1_dd << std::endl;
		std::cout << "S2Phi Value: " << mCalibratedData.Phi2_d << std::endl;
		std::cout << "S2XDD Value: " << mCalibratedData.X2_dd << std::endl;
		std::cout << "S2YDD Value: " << mCalibratedData.Y2_dd << std::endl;
		std::cout << "Phi Value [rad]: " << mstate.mPhi_C << std::endl;
		std::cout << "Phi Value [deg]: " << phi_disp << std::endl;

		float T_M = control();	//Calculating torque

		CCont.setContent(mADCVal, f, mdata1_raw, mdata2_raw, mstate);	//Write data in Container
		CCont.signalReader();	//Semaphore give

		//Control if phi in degree is between +-20 of 0
		if ((phi_disp < 20.0) && (phi_disp > -20.0))
		{
			mCBBHardware.setTorque(T_M);
			std::cout << "TM: " << T_M << std::endl;
		}
		else	//Else set torque to 0
		{
			mCBBHardware.setTorque(0.0);
			std::cout << "TM: " << 0.0 << std::endl;
		}
		time(&time_2);	//Time measurepoint 2
		usleep(20000 - (time_2 - time_1));	//Calculate delta t and sleeptime
	}
}
void CControlComp::init()
{
// init values
	mT = 0.02;					// Filtertime
	mrs1 = 0.14; 				// [m] Abstand Sen sor1 zur Drehachse
	mrs2 = 0.061; 				// [m] Abstand Sen sor2 zur Drehachse
	malpha_acc = mrs1 / mrs2; 	// Verhältnis für Berechnung des Winkels anhand der Beschleunigungsdaten
	mFilter_Koeff = 0.98; 		// [], Koeffizient für Komplementärfilter

// Kalibrierung
	Offset_mPhi1_d = -9.3482; 	// [], Offset des Gyrosensor1
	Offset_mPhi2_d = -6.0996; 	// [], Offset des Gyrosensor2
	gyro_gain = 1.0 / 16.8;		// Gyro gain
	adc_gain = 4.8975;			// ADC gain
	adc_offset = -9970.0;		// ADC offset
	X1_gain = -0.00059212;		// Sensor 1 X gain
	X1_off = 0.5021;			// Sensor 1 X offset
	X2_gain = -0.00059206;		// Sensor 2 X gain
	X2_off = -0.0305;			// Sensor 2 X offset
	Y1_gain = 0.00056873;		// Sensor 1 Y gain
	Y1_off = 0.4052;			// Sensor 1 Y offset
	Y2_gain = 0.00059528;		// Sensor 2 Y gain
	Y2_off = 0.1013;			// Sensor 2 Y offset

	Phi_C_old = 0.0;			// Phi value of previous itteration for filtering

	// Write values to class members
	mKalib_Obj.Gain_X1 = X1_gain;
	mKalib_Obj.Gain_Y1 = Y1_gain;
	mKalib_Obj.Gain_X2 = X2_gain;
	mKalib_Obj.Gain_Y2 = Y2_gain;
	mKalib_Obj.S0phi0 = Offset_mPhi1_d;
	mKalib_Obj.S0x0 = X1_off;
	mKalib_Obj.S0y0 = Y1_off;
	mKalib_Obj.S1x0 = X2_off;
	mKalib_Obj.S1y0 = Y2_off;
	mKalib_Obj.gain_phi = gyro_gain;
	mKalib_Obj.S1phi0 = Offset_mPhi2_d;
	mKalib_Obj.psi0 = adc_offset;
	mKalib_Obj.gain_psi = adc_gain;

	// Calculated control matrix
	KD1 = -1.5563;		// Phi
	KD2 = -0.2696;		// Phi_d
	KD3 = -0.00053255;	// Psi_d

}

void CControlComp::calibrate()
{ // inputs sind Sensor1Data/Sensor2Data mX__dd, mY__dd, mPhi__d, PSI__d_P2
// inputs sind x0, y0, alpha, gain .. aus dem KalibObjekt
// outputs sind SensorCalibData, kalibriert und geometrisiert
// CalculatedData mPhi__d_G, mPhi__dd_A, mPsi__d

	mCalibratedData.X1_dd = mKalib_Obj.Gain_X1 * mdata1_raw.mX__dd + mKalib_Obj.S0x0; // gain*( Wert + Offset))
	mCalibratedData.Y1_dd = mKalib_Obj.Gain_Y1 * mdata1_raw.mY__dd + mKalib_Obj.S0y0;
	mCalibratedData.X2_dd = mKalib_Obj.Gain_X2 * mdata2_raw.mX__dd + mKalib_Obj.S1x0;
	mCalibratedData.Y2_dd = mKalib_Obj.Gain_Y2 * mdata2_raw.mY__dd + mKalib_Obj.S1y0;
	mCalibratedData.Phi1_d = (mKalib_Obj.gain_phi * (mdata1_raw.mPhi__d + mKalib_Obj.S0phi0))
			* (3.14 / 180.0);
	mCalibratedData.Phi2_d = (mKalib_Obj.gain_phi * (mdata2_raw.mPhi__d + mKalib_Obj.S1phi0))
			* (3.14 / 180.0);
	mCalibratedData.Psi_d = (mKalib_Obj.gain_psi * mADCVal + mKalib_Obj.psi0) * (-1.0)
			* (2.0 * 3.141 / 60.0); // Kadc* (Wert + Offset_adc)
}

void CControlComp::filter()
{

	mstate.mPhi_A = atan(
			-(mCalibratedData.X1_dd - malpha_acc * mCalibratedData.X2_dd)
					/ (mCalibratedData.Y1_dd - malpha_acc * mCalibratedData.Y2_dd));
	mstate.mPhi_G = (((mCalibratedData.Phi1_d + mCalibratedData.Phi2_d) / 2.0));

	mstate.mPhi_C = ((mFilter_Koeff * (Phi_C_old + (mT * mstate.mPhi_G)))
			+ ((1.0 - mFilter_Koeff) * (mstate.mPhi_A+ PHI_OFF)));//+0.008
	Phi_C_old = mstate.mPhi_C;

}

float CControlComp::control()
{
	return ((KD1 * (mstate.mPhi_C ))
			+ (KD2 * (((mCalibratedData.Phi1_d + mCalibratedData.Phi2_d) / 2.0)))
			+ (KD3 * mCalibratedData.Psi_d)) * (-1.0);
}

