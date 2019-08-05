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

#ifndef DataGlobalConstants_hh_INCLUDED
#define DataGlobalConstants_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus.hh>

namespace EnergyPlus {

namespace DataGlobalConstants {

    // Data
    // MODULE PARAMETER DEFINITIONS:
    // End Use Parameters
    constexpr int NumEndUses(14);

    constexpr int endUseHeating(1);
    constexpr int endUseCooling(2);
    constexpr int endUseInteriorLights(3);
    constexpr int endUseExteriorLights(4);
    constexpr int endUseInteriorEquipment(5);
    constexpr int endUseExteriorEquipment(6);
    constexpr int endUseFans(7);
    constexpr int endUsePumps(8);
    constexpr int endUseHeatRejection(9);
    constexpr int endUseHumidification(10);
    constexpr int endUseHeatRecovery(11);
    constexpr int endUseWaterSystem(12);
    constexpr int endUseRefrigeration(13);
    constexpr int endUseCogeneration(14);

    // Resource Types
    extern std::string const cRT_None;
    extern std::string const cRT_NoneUC;
    constexpr int iRT_None(1000);
    extern std::string const cRT_Electricity;
    extern std::string const cRT_ElectricityUC;
    constexpr int iRT_Electricity(1001);
    extern std::string const cRT_Natural_Gas;
    extern std::string const cRT_Natural_GasUC;
    constexpr int iRT_Natural_Gas(1002);
    extern std::string const cRT_Gas;
    extern std::string const cRT_GasUC;
    constexpr int iRT_Gas(1002);
    extern std::string const cRT_Gasoline;
    extern std::string const cRT_GasolineUC;
    constexpr int iRT_Gasoline(1003);
    extern std::string const cRT_Diesel;
    extern std::string const cRT_DieselUC;
    constexpr int iRT_Diesel(1004);
    extern std::string const cRT_Coal;
    extern std::string const cRT_CoalUC;
    constexpr int iRT_Coal(1005);
    extern std::string const cRT_FuelOil_1;
    extern std::string const cRT_FuelOil_1UC;
    constexpr int iRT_FuelOil_1(1006);
    extern std::string const cRT_DistillateOil;
    extern std::string const cRT_DistillateOilUC;
    constexpr int iRT_DistillateOil(1006);
    extern std::string const cRT_FuelOil_2;
    extern std::string const cRT_FuelOil_2UC;
    constexpr int iRT_FuelOil_2(1007);
    extern std::string const cRT_ResidualOil;
    extern std::string const cRT_ResidualOilUC;
    constexpr int iRT_ResidualOil(1007);
    extern std::string const cRT_Propane;
    extern std::string const cRT_PropaneUC;
    constexpr int iRT_Propane(1008);
    extern std::string const cRT_LPG;
    extern std::string const cRT_LPGUC;
    constexpr int iRT_LPG(1008);
    extern std::string const cRT_Water;
    extern std::string const cRT_WaterUC;
    constexpr int iRT_Water(1009);
    extern std::string const cRT_EnergyTransfer;
    extern std::string const cRT_EnergyTransferUC;
    constexpr int iRT_EnergyTransfer(1010);
    extern std::string const cRT_Steam;
    extern std::string const cRT_SteamUC;
    constexpr int iRT_Steam(1011);
    extern std::string const cRT_DistrictCooling;
    extern std::string const cRT_DistrictCoolingUC;
    constexpr int iRT_DistrictCooling(1012);
    extern std::string const cRT_DistrictHeating;
    extern std::string const cRT_DistrictHeatingUC;
    constexpr int iRT_DistrictHeating(1013);
    extern std::string const cRT_ElectricityProduced;
    extern std::string const cRT_ElectricityProducedUC;
    constexpr int iRT_ElectricityProduced(1014);
    extern std::string const cRT_ElectricityPurchased;
    extern std::string const cRT_ElectricityPurchasedUC;
    constexpr int iRT_ElectricityPurchased(1015);
    extern std::string const cRT_ElectricitySurplusSold;
    extern std::string const cRT_ElectricitySurplusSoldUC;
    constexpr int iRT_ElectricitySurplusSold(1016);
    extern std::string const cRT_ElectricityNet;
    extern std::string const cRT_ElectricityNetUC;
    constexpr int iRT_ElectricityNet(1017);
    extern std::string const cRT_SolarWater;
    extern std::string const cRT_SolarWaterUC;
    constexpr int iRT_SolarWater(1018);
    extern std::string const cRT_SolarAir;
    extern std::string const cRT_SolarAirUC;
    constexpr int iRT_SolarAir(1019);
    extern std::string const cRT_SO2;
    extern std::string const cRT_SO2UC;
    constexpr int iRT_SO2(1020);
    extern std::string const cRT_NOx;
    extern std::string const cRT_NOxUC;
    constexpr int iRT_NOx(1021);
    extern std::string const cRT_N2O;
    extern std::string const cRT_N2OUC;
    constexpr int iRT_N2O(1022);
    extern std::string const cRT_PM;
    extern std::string const cRT_PMUC;
    constexpr int iRT_PM(1023);
    extern std::string const cRT_PM2_5;
    extern std::string const cRT_PM2_5UC;
    constexpr int iRT_PM2_5(1024);
    extern std::string const cRT_PM10;
    extern std::string const cRT_PM10UC;
    constexpr int iRT_PM10(1025);
    extern std::string const cRT_CO;
    extern std::string const cRT_COUC;
    constexpr int iRT_CO(1026);
    extern std::string const cRT_CO2;
    extern std::string const cRT_CO2UC;
    constexpr int iRT_CO2(1027);
    extern std::string const cRT_CH4;
    extern std::string const cRT_CH4UC;
    constexpr int iRT_CH4(1028);
    extern std::string const cRT_NH3;
    extern std::string const cRT_NH3UC;
    constexpr int iRT_NH3(1029);
    extern std::string const cRT_NMVOC;
    extern std::string const cRT_NMVOCUC;
    constexpr int iRT_NMVOC(1030);
    extern std::string const cRT_Hg;
    extern std::string const cRT_HgUC;
    constexpr int iRT_Hg(1031);
    extern std::string const cRT_Pb;
    extern std::string const cRT_PbUC;
    constexpr int iRT_Pb(1032);
    extern std::string const cRT_NuclearHigh;
    extern std::string const cRT_NuclearHighUC;
    constexpr int iRT_NuclearHigh(1033);
    extern std::string const cRT_NuclearLow;
    extern std::string const cRT_NuclearLowUC;
    constexpr int iRT_NuclearLow(1034);
    extern std::string const cRT_WaterEnvironmentalFactors;
    extern std::string const cRT_WaterEnvironmentalFactorsUC;
    constexpr int iRT_WaterEnvironmentalFactors(1035);
    extern std::string const cRT_CarbonEquivalent;
    extern std::string const cRT_CarbonEquivalentUC;
    constexpr int iRT_CarbonEquivalent(1036);
    extern std::string const cRT_Source;
    extern std::string const cRT_SourceUC;
    constexpr int iRT_Source(1037);
    extern std::string const cRT_PlantLoopHeatingDemand;
    extern std::string const cRT_PlantLoopHeatingDemandUC;
    constexpr int iRT_PlantLoopHeatingDemand(1038);
    extern std::string const cRT_PlantLoopCoolingDemand;
    extern std::string const cRT_PlantLoopCoolingDemandUC;
    constexpr int iRT_PlantLoopCoolingDemand(1039);
    extern std::string const cRT_OnSiteWater;
    extern std::string const cRT_OnSiteWaterUC;
    constexpr int iRT_OnSiteWater(1040);
    extern std::string const cRT_MainsWater;
    extern std::string const cRT_MainsWaterUC;
    constexpr int iRT_MainsWater(1041);
    extern std::string const cRT_RainWater;
    extern std::string const cRT_RainWaterUC;
    constexpr int iRT_RainWater(1042);
    extern std::string const cRT_WellWater;
    extern std::string const cRT_WellWaterUC;
    constexpr int iRT_WellWater(1043);
    extern std::string const cRT_Condensate;
    extern std::string const cRT_CondensateUC;
    constexpr int iRT_Condensate(1044);
    extern std::string const cRT_OtherFuel1;
    extern std::string const cRT_OtherFuel1UC;
    constexpr int iRT_OtherFuel1(1045);
    extern std::string const cRT_OtherFuel2;
    extern std::string const cRT_OtherFuel2UC;
    constexpr int iRT_OtherFuel2(1046);
    constexpr int NumOfResourceTypes(46);
    constexpr int ResourceTypeInitialOffset(1000); // to reach "ValidTypes"
    extern Array1D_string const cRT_ValidTypes;

    constexpr int iGeneratorICEngine(1);
    constexpr int iGeneratorCombTurbine(2);
    constexpr int iGeneratorPV(3);
    constexpr int iGeneratorFuelCell(4);
    constexpr int iGeneratorMicroCHP(5);
    constexpr int iGeneratorMicroturbine(6);
    constexpr int iGeneratorWindTurbine(7);
    constexpr int iGeneratorPVWatts(8);

    constexpr int iEvapCoolerDirectCELDEKPAD(1001);
    constexpr int iEvapCoolerInDirectCELDEKPAD(1002);
    constexpr int iEvapCoolerInDirectWETCOIL(1003);
    constexpr int iEvapCoolerInDirectRDDSpecial(1004);
    constexpr int iEvapCoolerDirectResearchSpecial(1005);


    // DERIVED TYPE DEFINITIONS:
    // na

    // MODULE VARIABLE DECLARATIONS:
    // na

    // SUBROUTINE SPECIFICATIONS FOR MODULE DataGlobalConstants

    // Functions

    int AssignResourceTypeNum(std::string const &ResourceTypeChar);

    std::string GetResourceTypeChar(int const ResourceTypeNum);

} // namespace DataGlobalConstants

} // namespace EnergyPlus

#endif
