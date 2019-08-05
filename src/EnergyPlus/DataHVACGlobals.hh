// EnergyPlus, Copyright (c) 1996-2019, The Board of Trustees of the University of Illinois,
// The Regents of the University of California, through Lawrence Berkeley National Laboratory
// (subject to receipt of any required approvals from the U.S. Dept. of Energy), Oak Ridge
// National Laboratory, managed by UT-Battelle, Alliance for Sustainable Energy, LLC, and other
// contributors. All rights reserved.
//
// NOTICE: This Software was developed under funding from the U.S. Department of Energy and the
// U.S. Government consequently retains certain rights. As such, the U.S. Government has been
// granted for itself and others acting on its behalf a paid-up, nonexclusive, irrevocable,
// worldwide license in the Software to reproduce, distribute copies to the public, prepare
// derivative works, and perform publicly and display publicly, and to permit others to do so.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
// (1) Redistributions of source code must retain the above copyright notice, this list of
//     conditions and the following disclaimer.
//
// (2) Redistributions in binary form must reproduce the above copyright notice, this list of
//     conditions and the following disclaimer in the documentation and/or other materials
//     provided with the distribution.
//
// (3) Neither the name of the University of California, Lawrence Berkeley National Laboratory,
//     the University of Illinois, U.S. Dept. of Energy nor the names of its contributors may be
//     used to endorse or promote products derived from this software without specific prior
//     written permission.
//
// (4) Use of EnergyPlus(TM) Name. If Licensee (i) distributes the software in stand-alone form
//     without changes from the version obtained under this License, or (ii) Licensee makes a
//     reference solely to the software portion of its product, Licensee must refer to the
//     software as "EnergyPlus version X" software, where "X" is the version number Licensee
//     obtained under this License and may not use a different name for the software. Except as
//     specifically required in this Section (4), Licensee shall not use in a company name, a
//     product name, in advertising, publicity, or other promotional activities any name, trade
//     name, trademark, logo, or other designation of "EnergyPlus", "E+", "e+" or confusingly
//     similar designation, without the U.S. Department of Energy's prior written consent.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
// AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef DataHVACGlobals_hh_INCLUDED
#define DataHVACGlobals_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <DataGlobals.hh>
#include <EnergyPlus.hh>

namespace EnergyPlus {

namespace DataHVACGlobals {

    // Using/Aliasing

    // Data
    // -only module should be available to other modules and routines.
    // Thus, all variables in this module must be PUBLIC.
    enum class HVACSystemRootSolverAlgorithm : int
    {
        RegulaFalsi = 0,
        Bisection,
        RegulaFalsiThenBisection,
        BisectionThenRegulaFalsi,
        Alternation
    };

    // MODULE PARAMETER DEFINITIONS:

    constexpr Real64 SmallTempDiff(1.0E-5);
    constexpr Real64 SmallMassFlow(0.001);
    constexpr Real64 VerySmallMassFlow(1.0E-30);
    constexpr Real64 SmallLoad(1.0);
    constexpr Real64 TempControlTol(0.1); // temperature control tolerance for packaged equip. [deg C]
    constexpr Real64 SmallAirVolFlow(0.001);
    constexpr Real64 SmallWaterVolFlow(1.0E-9);
    constexpr Real64 BlankNumeric(-99999.0);      // indicates numeric input field was blank
    constexpr Real64 RetTempMax(60.0);            // maximum return air temperature [deg C]
    constexpr Real64 RetTempMin(-30.0);           // minimum return air temperature [deg C]
    constexpr Real64 DesCoilHWInletTempMin(46.0); // minimum heating water coil water inlet temp for UA sizing only. [deg C]

    // Number of Sizing types from list below
    constexpr int NumOfSizingTypes(33); // number of sizing types

    // Sizing types
    constexpr int CoolingAirflowSizing(1);                               // request sizing for cooling air flow rate
    constexpr int CoolingWaterflowSizing(2);                             // request sizing for cooling water flow rate
    constexpr int HeatingWaterflowSizing(3);                             // request sizing for heating coil water flow rate
    constexpr int CoolingWaterDesAirInletTempSizing(4);                  // request sizing for cooling water coil inlet air temp
    constexpr int CoolingWaterDesAirInletHumRatSizing(5);                // request sizing for cooling water coil inlet air humidity ratio
    constexpr int CoolingWaterDesWaterInletTempSizing(6);                // request sizing for cooling water coil inlet water temp
    constexpr int CoolingWaterDesAirOutletTempSizing(7);                 // request sizing for cooling water coil outlet air temp
    constexpr int CoolingWaterDesAirOutletHumRatSizing(8);               // request sizing for cooling water coil outlet air humidity ratio
    constexpr int CoolingWaterNumofTubesPerRowSizing(9);                 // request sizing for cooling water coil number of tubes per row
    constexpr int HeatingWaterDesAirInletTempSizing(10);                 // request sizing for heating water coil inlet air temp
    constexpr int HeatingWaterDesAirInletHumRatSizing(11);               // request sizing for heating water coil inlet air humidity ratio
    constexpr int HeatingWaterDesCoilLoadUsedForUASizing(12);            // request sizing for heating water coil capacity used for UA sizing
    constexpr int HeatingWaterDesCoilWaterVolFlowUsedForUASizing(13);    // request sizing for heating water coil volume flow rate used for UA sizing
    constexpr int HeatingAirflowSizing(14);                              // request sizing for heating air flow rate
    constexpr int HeatingAirflowUASizing(15);                            // request sizing for heating air flow rate
    constexpr int SystemAirflowSizing(16);                               // request sizing for system air flow rate
    constexpr int CoolingCapacitySizing(17);                             // request sizing for cooling capacity
    constexpr int HeatingCapacitySizing(18);                             // request sizing for heating capacity
    constexpr int WaterHeatingCapacitySizing(19);                        // request sizing for water-side heating capacity
    constexpr int WaterHeatingCoilUASizing(20);                          // request sizing for heating coil UA
    constexpr int SystemCapacitySizing(21);                              // request sizing for system capacity
    constexpr int CoolingSHRSizing(22);                                  // request sizing for cooling SHR
    constexpr int HeatingDefrostSizing(23);                              // request sizing for heating defrost capacity
    constexpr int MaxHeaterOutletTempSizing(24);                         // request sizing for heating coil maximum outlet temperature
    constexpr int AutoCalculateSizing(25);                               // identifies an autocalulate input
    constexpr int ZoneCoolingLoadSizing(26);                             // zone cooling sensible load (zsz file)
    constexpr int ZoneHeatingLoadSizing(27);                             // zome heating sensible load (zsz file)
    constexpr int MinSATempCoolingSizing(28);                            // minimum SA temperature in cooling
    constexpr int MaxSATempHeatingSizing(29);                            // maximum SA temperature in heating
    constexpr int ASHRAEMinSATCoolingSizing(30);                         // minimum SA temperature in cooling model when using ASHRAE 90.1 SZVAV method
    constexpr int ASHRAEMaxSATHeatingSizing(31);                         // maximum SA temperature in heating model when using ASHRAE 90.1 SZVAV method
    constexpr int HeatingCoilDesAirInletTempSizing(32);                  // design inlet air temperature for heating coil
    constexpr int HeatingCoilDesAirOutletTempSizing(33);                 // design outlet air temperature for heating coil
    constexpr int HeatingCoilDesAirInletHumRatSizing(34);                // design inlet air humidity ratio for heating coil
    constexpr int DesiccantDehumidifierBFPerfDataFaceVelocitySizing(35); // identifies desiccant performance data face velocity autosisizing input

    // Condenser Type (using same numbering scheme as for chillers)
    constexpr int AirCooled(1);   // Air-cooled condenser
    constexpr int WaterCooled(2); // Water-cooled condenser
    constexpr int EvapCooled(3);  // Evaporatively-cooled condenser
    constexpr int WaterHeater(4); // Condenser heats water (e.g., in water heater tank)

    // The following parameters are used for system availability status
    constexpr int NoAction(0);
    constexpr int ForceOff(1);
    constexpr int CycleOn(2);
    constexpr int CycleOnZoneFansOnly(3);
    // The following parameters describe the setpoint types in TempControlType(ActualZoneNum)
    constexpr int SingleHeatingSetPoint(1);
    constexpr int SingleCoolingSetPoint(2);
    constexpr int SingleHeatCoolSetPoint(3);
    constexpr int DualSetPointWithDeadBand(4);
    // parameters describing air duct type
    constexpr int Main(1);
    constexpr int Cooling(2);
    constexpr int Heating(3);
    constexpr int Other(4);
    constexpr int RAB(5);
    // parameters describing fan types
    constexpr int NumAllFanTypes(6); // cpw22Aug2010 (was 4)

    // fan types
    constexpr int FanType_SimpleConstVolume(1);
    constexpr int FanType_SimpleVAV(2);
    constexpr int FanType_SimpleOnOff(3);
    constexpr int FanType_ZoneExhaust(4);
    constexpr int FanType_ComponentModel(5);    // cpw22Aug2010
    constexpr int FanType_SystemModelObject(6); // new for V8.7, simple versatile fan object

    // Fan Minimum Flow Fraction Input Method
    constexpr int MinFrac(1);
    constexpr int FixedMin(2);
    // Fan mode
    constexpr int CycFanCycCoil(1);  // Cycling fan, cycling coil = 1
    constexpr int ContFanCycCoil(2); // Continuous fan, cycling coil = 2
    // Fan placement
    constexpr int BlowThru(1); // fan before coil
    constexpr int DrawThru(2); // fan after coil
    // OA Controller Heat Recovery Bypass Control Types
    constexpr int BypassWhenWithinEconomizerLimits(0);   // heat recovery controlled by economizer limits
    constexpr int BypassWhenOAFlowGreaterThanMinimum(1); // heat recovery ON at minimum OA in economizer mode

    extern Array1D_string const cFanTypes; // cpw22Aug2010 | cpw22Aug2010 (new)

    // parameters describing unitary systems
    constexpr int NumUnitarySystemTypes(7);
    // Furnace/Unitary System Types
    constexpr int Furnace_HeatOnly(1);
    constexpr int Furnace_HeatCool(2);
    constexpr int UnitarySys_HeatOnly(3);
    constexpr int UnitarySys_HeatCool(4);
    constexpr int UnitarySys_HeatPump_AirToAir(5);
    constexpr int UnitarySys_HeatPump_WaterToAir(6);
    constexpr int UnitarySys_AnyCoilType(7);
    extern Array1D_string const cFurnaceTypes;

    // parameters describing coil types
    // parameters describing coil types
    constexpr int NumAllCoilTypes(34);

    constexpr int CoilDX_CoolingSingleSpeed(1);
    constexpr int CoilDX_HeatingEmpirical(2);
    constexpr int CoilDX_CoolingTwoSpeed(3);
    constexpr int CoilDX_CoolingHXAssisted(4);
    constexpr int CoilDX_CoolingTwoStageWHumControl(5);
    constexpr int CoilDX_HeatPumpWaterHeaterPumped(6);
    constexpr int CoilDX_HeatPumpWaterHeaterWrapped(7);
    constexpr int CoilDX_MultiSpeedCooling(8);
    constexpr int CoilDX_MultiSpeedHeating(9);

    constexpr int Coil_HeatingGasOrOtherFuel(10);
    constexpr int Coil_HeatingGas_MultiStage(11);
    constexpr int Coil_HeatingElectric(12);
    constexpr int Coil_HeatingElectric_MultiStage(13);
    constexpr int Coil_HeatingDesuperheater(14);

    constexpr int Coil_CoolingWater(15);
    constexpr int Coil_CoolingWaterDetailed(16);
    constexpr int Coil_HeatingWater(17);
    constexpr int Coil_HeatingSteam(18);
    constexpr int CoilWater_CoolingHXAssisted(19);

    constexpr int Coil_CoolingWaterToAirHP(20);
    constexpr int Coil_HeatingWaterToAirHP(21);
    constexpr int Coil_CoolingWaterToAirHPSimple(22);
    constexpr int Coil_HeatingWaterToAirHPSimple(23);
    constexpr int CoilVRF_Cooling(24);
    constexpr int CoilVRF_Heating(25);

    constexpr int Coil_UserDefined(26);
    constexpr int CoilDX_PackagedThermalStorageCooling(27);

    constexpr int Coil_CoolingWaterToAirHPVSEquationFit(28);
    constexpr int Coil_HeatingWaterToAirHPVSEquationFit(29);
    constexpr int Coil_CoolingAirToAirVariableSpeed(30);
    constexpr int Coil_HeatingAirToAirVariableSpeed(31);
    constexpr int CoilDX_HeatPumpWaterHeaterVariableSpeed(32);

    constexpr int CoilVRF_FluidTCtrl_Cooling(33);
    constexpr int CoilVRF_FluidTCtrl_Heating(34);

    extern Array1D_string const cAllCoilTypes;
    extern Array1D_string const cCoolingCoilTypes;
    extern Array1D_string const cHeatingCoilTypes;

    // Water to air HP coil types
    constexpr int WatertoAir_Simple(1);
    constexpr int WatertoAir_ParEst(2);
    constexpr int WatertoAir_VarSpeedEquationFit(3);
    constexpr int WatertoAir_VarSpeedLooUpTable(4);

    // Water to Air HP Water Flow Mode
    constexpr int WaterCycling(1);  // water flow cycles with compressor
    constexpr int WaterConstant(2); // water flow is constant
    constexpr int
        WaterConstantOnDemand(3); // water flow is constant whenever the coil is operational - this is the only method used in EP V7.2 and earlier

    // parameters describing coil performance types
    constexpr int CoilPerfDX_CoolBypassEmpirical(100);

    // Airflow per total capacity range (Regular DX coils)
    constexpr Real64 MaxRatedVolFlowPerRatedTotCap1(0.00006041); // m3/s per watt = 450 cfm/ton
    constexpr Real64 MinRatedVolFlowPerRatedTotCap1(0.00004027); // m3/s per watt = 300 cfm/ton
    constexpr Real64 MaxHeatVolFlowPerRatedTotCap1(0.00008056);  // m3/s per watt = 600 cfm/ton
    constexpr Real64 MaxCoolVolFlowPerRatedTotCap1(0.00006713);  // m3/s per watt = 500 cfm/ton
    constexpr Real64 MinOperVolFlowPerRatedTotCap1(0.00002684);  // m3/s per watt = 200 cfm/ton

    // 100% DOAS DX coils Airflow per total capacity ratio
    constexpr Real64 MaxRatedVolFlowPerRatedTotCap2(0.00003355); // m3/s per watt = 250 cfm/ton
    constexpr Real64 MinRatedVolFlowPerRatedTotCap2(0.00001677); // m3/s per watt = 125 cfm/ton
    constexpr Real64 MaxHeatVolFlowPerRatedTotCap2(0.00004026);  // m3/s per watt = 300 cfm/ton
    constexpr Real64 MaxCoolVolFlowPerRatedTotCap2(0.00004026);  // m3/s per watt = 300 cfm/ton
    constexpr Real64 MinOperVolFlowPerRatedTotCap2(0.00001342);  // m3/s per watt = 100 cfm/ton

    extern Array1D<Real64> MaxRatedVolFlowPerRatedTotCap;
    extern Array1D<Real64> MinRatedVolFlowPerRatedTotCap;
    extern Array1D<Real64> MaxHeatVolFlowPerRatedTotCap;
    extern Array1D<Real64> MaxCoolVolFlowPerRatedTotCap;
    extern Array1D<Real64> MinOperVolFlowPerRatedTotCap;

    // dx coil type (DXCT)
    constexpr int RegularDXCoil(1); // Regular DX coils or mixed air dx coils
    constexpr int DOASDXCoil(2);    // 100% DOAS DX coils
    extern int DXCT;                // dx coil type: regular DX coil ==1, 100% DOAS DX coil = 2

    // Parameters describing Heat Exchanger types
    constexpr int NumHXTypes(3);

    constexpr int HX_AIRTOAIR_FLATPLATE(1);
    constexpr int HX_AIRTOAIR_GENERIC(2);
    constexpr int HX_DESICCANT_BALANCED(3);

    extern Array1D_string const cHXTypes;

    // Parameters describing air terminal mixers
    constexpr int NumATMixerTypes(2);

    constexpr int No_ATMixer(0);
    constexpr int ATMixer_InletSide(1);
    constexpr int ATMixer_SupplySide(2);

    extern Array1D_string const cATMixerTypes;
    constexpr bool ATMixerExists(true);

    // Parameters describing variable refrigerant flow terminal unit types
    constexpr int NumVRFTUTypes(1);

    constexpr int VRFTUType_ConstVolume(1);

    extern Array1D_string const cVRFTUTypes;

    // VRF Heating Performance Curve Temperature Type
    constexpr int NumVRFHeatingPerformanceOATTypes(2);
    constexpr int WetBulbIndicator(1);
    constexpr int DryBulbIndicator(2);

    extern Array1D_string const cVRFHeatingPerformanceOATTypes;

    // parameter concerning the amount of change in zone temperature is needed
    // for oscillation of zone temperature to be detected.
    constexpr Real64 OscillateMagnitude(0.15);

    // Parameters for HVACSystemRootFindingAlgorithm
    constexpr int Bisection(2);

    // DERIVED TYPE DEFINITIONS

    // INTERFACE BLOCK SPECIFICATIONS

    // MODULE VARIABLE DECLARATIONS:

    extern bool FirstTimeStepSysFlag; // Set to true at the start of each sub-time step

    extern Real64 TimeStepSys;                    // System Time Increment - the adaptive time step used by the HVAC simulation (hours)
    extern Real64 SysTimeElapsed;                 // elapsed system time in zone timestep (hours)
    extern Real64 FracTimeStepZone;               // System time step divided by the zone time step
    extern bool ShortenTimeStepSys;               // Logical flag that triggers shortening of system time step
    extern int NumOfSysTimeSteps;                 // for current zone time step, number of system timesteps inside  it
    extern int NumOfSysTimeStepsLastZoneTimeStep; // previous zone time step, num of system timesteps inside
    extern int LimitNumSysSteps;

    extern bool UseZoneTimeStepHistory;     // triggers use of zone time step history, else system time step history, for ZTM1, ZTMx
    extern int NumPlantLoops;               // Number of plant loops specified in simulation
    extern int NumCondLoops;                // Number of condenser plant loops specified in simulation
    extern int NumElecCircuits;             // Number of electric circuits specified in simulation
    extern int NumGasMeters;                // Number of gas meters specified in simulation
    extern int NumPrimaryAirSys;            // Number of primary HVAC air systems
    extern Real64 OnOffFanPartLoadFraction; // fan part-load fraction (Fan:OnOff)
    extern Real64 DXCoilTotalCapacity;      // DX coil total cooling capacity (eio report var for HPWHs)
    extern Real64 DXElecCoolingPower;       // Electric power consumed by DX cooling coil last DX simulation
    extern Real64 DXElecHeatingPower;       // Electric power consumed by DX heating coil last DX simulation
    extern Real64 ElecHeatingCoilPower;     // Electric power consumed by electric heating coil
    extern Real64 AirToAirHXElecPower;      // Electric power consumed by Heat Exchanger:Air To Air (Generic or Flat Plate)
    // from last simulation in HeatRecovery.cc
    extern Real64 UnbalExhMassFlow;      // unbalanced zone exhaust from a zone equip component [kg/s]
    extern Real64 BalancedExhMassFlow;   // balanced zone exhaust (declared as so by user)  [kg/s]
    extern Real64 PlenumInducedMassFlow; // secondary air mass flow rate induced from a return plenum [kg/s]
    extern bool TurnFansOn;              // If true overrides fan schedule and cycles fans on
    extern bool TurnZoneFansOnlyOn; // If true overrides zone fan schedule and cycles fans on (currently used only by parallel powered induction unit)
    extern bool TurnFansOff;        // If True overides fan schedule and TurnFansOn and forces fans off
    extern bool ZoneCompTurnFansOn; // If true overrides fan schedule and cycles fans on
    extern bool ZoneCompTurnFansOff; // If True overides fan schedule and TurnFansOn and forces fans off
    extern bool SetPointErrorFlag;   // True if any needed setpoints not set; if true, program terminates
    extern bool DoSetPointTest;      // True one time only for sensed node setpoint test
    extern bool NightVentOn;         // set TRUE in SimAirServingZone if night ventilation is happening

    extern int NumTempContComps;
    extern Real64 HPWHInletDBTemp;     // Used by curve objects when calculating DX coil performance for HEAT PUMP:WATER HEATER
    extern Real64 HPWHInletWBTemp;     // Used by curve objects when calculating DX coil performance for HEAT PUMP:WATER HEATER
    extern Real64 HPWHCrankcaseDBTemp; // Used for HEAT PUMP:WATER HEATER crankcase heater ambient temperature calculations
    extern bool AirLoopInit;           // flag for whether InitAirLoops has been called
    extern bool AirLoopsSimOnce;       // True means that the air loops have been simulated once in this environment
    extern bool GetAirPathDataDone;    // True means that air loops inputs have been processed

    // Hybrid ventilation control part
    extern int NumHybridVentSysAvailMgrs;                  // Number of hybrid ventilation control
    extern Array1D_int HybridVentSysAvailAirLoopNum;       // Airloop number in hybrid vent availability manager
    extern Array1D_int HybridVentSysAvailVentCtrl;         // Ventilation control action in hybrid vent availability manager
    extern Array1D_int HybridVentSysAvailActualZoneNum;    // Actual zone num in hybrid vent availability manager
    extern Array1D_int HybridVentSysAvailANCtrlStatus;     // AN control status in hybrid vent availability manager
    extern Array1D_int HybridVentSysAvailMaster;           // Master object name: Ventilation for simple; Zone name for AN
    extern Array1D<Real64> HybridVentSysAvailWindModifier; // Wind modifier for AirflowNetwork
    // For multispeed heat pump only
    extern Real64 MSHPMassFlowRateLow;     // Mass flow rate at low speed
    extern Real64 MSHPMassFlowRateHigh;    // Mass flow rate at high speed
    extern Real64 MSHPWasteHeat;           // Waste heat
    extern Real64 PreviousTimeStep;        // The time step length at the previous time step
    extern bool ShortenTimeStepSysRoomAir; // Logical flag that triggers shortening of system time step

    extern Real64 deviationFromSetPtThresholdHtg; // heating threshold for reporting setpoint deviation
    extern Real64 deviationFromSetPtThresholdClg; // cooling threshold for reporting setpoint deviation

    extern bool SimAirLoopsFlag;               // True when the air loops need to be (re)simulated
    extern bool SimElecCircuitsFlag;           // True when electic circuits need to be (re)simulated
    extern bool SimPlantLoopsFlag;             // True when the main plant loops need to be (re)simulated
    extern bool SimZoneEquipmentFlag;          // True when zone equipment components need to be (re)simulated
    extern bool SimNonZoneEquipmentFlag;       // True when non-zone equipment components need to be (re)simulated
    extern bool ZoneMassBalanceHVACReSim;      // True when zone air mass flow balance and air loop needs (re)simulated
    extern int MinAirLoopIterationsAfterFirst; // minimum number of HVAC iterations after FirstHVACIteration (must be at least 2 for sequenced loads
                                               // to operate on air loops)

    constexpr int NumZoneHVACTerminalTypes(38);
    extern Array1D_string const ccZoneHVACTerminalTypes;
    extern Array1D_string const ZoneHVACTerminalTypes;
    constexpr int ZoneEquipTypeOf_VariableRefrigerantFlow(1);
    constexpr int ZoneEquipTypeOf_EnergyRecoveryVentilator(2);
    constexpr int ZoneEquipTypeOf_FourPipeFanCoil(3);
    constexpr int ZoneEquipTypeOf_OutdoorAirUnit(4);
    constexpr int ZoneEquipTypeOf_PackagedTerminalAirConditioner(5);
    constexpr int ZoneEquipTypeOf_PackagedTerminalHeatPump(6);
    constexpr int ZoneEquipTypeOf_UnitHeater(7);
    constexpr int ZoneEquipTypeOf_UnitVentilator(8);
    constexpr int ZoneEquipTypeOf_VentilatedSlab(9);
    constexpr int ZoneEquipTypeOf_WaterToAirHeatPump(10);
    constexpr int ZoneEquipTypeOf_WindowAirConditioner(11);
    constexpr int ZoneEquipTypeOf_BaseboardRadiantConvectiveElectric(12);
    constexpr int ZoneEquipTypeOf_BaseboardRadiantConvectiveWater(13);
    constexpr int ZoneEquipTypeOf_BaseboardRadiantConvectiveSteam(14);
    constexpr int ZoneEquipTypeOf_BaseboardConvectiveElectric(15);
    constexpr int ZoneEquipTypeOf_BaseboardConvectiveWater(16);
    constexpr int ZoneEquipTypeOf_HighTemperatureRadiant(17);
    constexpr int ZoneEquipTypeOf_DehumidifierDX(18);
    constexpr int ZoneEquipTypeOf_IdealLoadsAirSystem(19);
    constexpr int ZoneEquipTypeOf_RefrigerationChillerSet(20);
    constexpr int ZoneEquipTypeOf_FanZoneExhaust(21);
    constexpr int ZoneEquipTypeOf_WaterHeaterHeatPump(22);
    constexpr int ZoneEquipTypeOf_AirTerminalSingleDuctUncontrolled(23);
    constexpr int ZoneEquipTypeOf_AirTerminalDualDuctConstantVolume(24);
    constexpr int ZoneEquipTypeOf_AirTerminalDualDuctVAV(25);
    constexpr int ZoneEquipTypeOf_AirTerminalSingleDuctConstantVolumeReheat(26);
    constexpr int ZoneEquipTypeOf_AirTerminalSingleDuctConstantVolumeNoReheat(27);
    constexpr int ZoneEquipTypeOf_AirTerminalSingleDuctVAVReheat(28);
    constexpr int ZoneEquipTypeOf_AirTerminalSingleDuctVAVNoReheat(29);
    constexpr int ZoneEquipTypeOf_AirTerminalSingleDuctSeriesPIUReheat(30);
    constexpr int ZoneEquipTypeOf_AirTerminalSingleDuctParallelPIUReheat(31);
    constexpr int ZoneEquipTypeOf_AirTerminalSingleDuctCAVFourPipeInduction(32);
    constexpr int ZoneEquipTypeOf_AirTerminalSingleDuctVAVReheatVariableSpeedFan(33);
    constexpr int ZoneEquipTypeOf_AirTerminalSingleDuctVAVHeatAndCoolReheat(34);
    constexpr int ZoneEquipTypeOf_AirTerminalSingleDuctVAVHeatAndCoolNoReheat(35);
    constexpr int ZoneEquipTypeOf_AirTerminalSingleDuctConstantVolumeCooledBeam(36);
    constexpr int ZoneEquipTypeOf_AirTerminalDualDuctVAVOutdoorAir(37);
    constexpr int ZoneEquipTypeOf_AirLoopHVACReturnAir(38);


    // Types

    struct ComponentSetPtData
    {
        // Members
        // CHARACTER(len=MaxNameLength) :: EquipOperListName
        std::string EquipmentType;
        std::string EquipmentName;
        int NodeNumIn;
        int NodeNumOut;
        Real64 EquipDemand;
        Real64 DesignFlowRate;
        std::string HeatOrCool;
        int OpType;

        // Default Constructor
        ComponentSetPtData() : NodeNumIn(0), NodeNumOut(0), EquipDemand(0.0), DesignFlowRate(0.0), OpType(0)
        {
        }
    };

    struct DefineZoneCompAvailMgrs
    {
        // Members
        int NumAvailManagers;             // number of availability managers for this system
        int AvailStatus;                  // system availability status
        int StartTime;                    // cycle on time (in SimTimeSteps)
        int StopTime;                     // cycle off time (in SimTimeSteps)
        std::string AvailManagerListName; // name of each availability manager
        Array1D_string AvailManagerName;  // name of each availability manager
        Array1D_int AvailManagerType;     // type of availability manager
        Array1D_int AvailManagerNum;      // index for availability manager
        int ZoneNum;                      // cycle off time (in SimTimeSteps)
        bool Input;                       // starts off as true to initialize zone equipment availability manager data
        int Count;                        // initialize twice to ensure zone equipment availability manager list name has been read in

        // Default Constructor
        DefineZoneCompAvailMgrs() : NumAvailManagers(0), AvailStatus(0), StartTime(0), StopTime(0), ZoneNum(0), Input(true), Count(0)
        {
        }
    };

    struct ZoneCompTypeData
    {
        // Members
        Array1D<DefineZoneCompAvailMgrs> ZoneCompAvailMgrs;
        int TotalNumComp; // total number of components of a zone equip type

        // Default Constructor
        ZoneCompTypeData() : TotalNumComp(0)
        {
        }
    };

    struct OptStartDataType
    {
        // Members
        Array1D_int ActualZoneNum;
        Array1D<Real64> OccStartTime;
        Array1D_bool OptStartFlag;

        // Default Constructor
        OptStartDataType()
        {
        }
    };

    struct HVACSystemRootFindingAlgorithm
    {
        // Members
        std::string Algorithm;                              // Choice of algorithm
        int NumOfIter;                                      // Number of Iteration Before Algorith Switch
        HVACSystemRootSolverAlgorithm HVACSystemRootSolver; // 1 RegulaFalsi; 2 Bisection; 3 BisectionThenRegulaFalsi; 4 RegulaFalsiThenBisection; 5
                                                            // Alternation Default Constructor
        HVACSystemRootFindingAlgorithm() : NumOfIter(5), HVACSystemRootSolver(HVACSystemRootSolverAlgorithm::RegulaFalsi)
        {
        }
    };

    // Object Data
    extern Array1D<ZoneCompTypeData> ZoneComp;
    extern OptStartDataType OptStartData; // For optimum start
    extern Array1D<ComponentSetPtData> CompSetPtEquip;
    extern HVACSystemRootFindingAlgorithm HVACSystemRootFinding;

    // Clears the global data in DataHVACGlobals.
    // Needed for unit tests, should not be normally called.
    void clear_state();

} // namespace DataHVACGlobals

} // namespace EnergyPlus

#endif
