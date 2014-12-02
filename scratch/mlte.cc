#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include <ns3/buildings-helper.h>

using namespace ns3;

int main(int argc, char* argv[]){
    
    uint32_t nUe = 6;
    uint32_t nEnb = 2;
    double simTime = 2.0;
    
    unsigned int i = 0;
    
    CommandLine cmd;
    cmd.AddValue ("nUe", "Number of User Equipment nodes/devices", nUe);
    cmd.AddValue ("nEnb", "Number of Evolved Node B nodes/devices", nEnb);
    cmd.AddValue ("simTime", "Total max duration of simulation in seconds", simTime);
    cmd.Parse (argc, argv);
    if (nUe<nEnb)
        nUe=nEnb;
    if (nUe<1)
        nUe=1;
    if (nEnb<1)
        nEnb=1;
    if (simTime<1)
        simTime=1.0;
    
    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();
    
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
    
    NodeContainer enbNodes;
    NodeContainer ueNodes[nEnb];
    enbNodes.Create (nEnb);
    for (i=0;i<nEnb-1;i++)
        ueNodes[i].Create (nUe/nEnb);
    ueNodes[nEnb-1].Create (nUe-((nUe/nEnb)*(nEnb-1)));
    
    MobilityHelper mobility;
    mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                   "MinX", DoubleValue (0.0),
                                   "MinY", DoubleValue (0.0),
                                   "DeltaX", DoubleValue (5.0),
                                   "DeltaY", DoubleValue (10.0),
                                   "GridWidth", UintegerValue (3),
                                   "LayoutType", StringValue ("RowFirst"));

    mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                               "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));
    for (i=0;i<nEnb;i++){
        mobility.Install (ueNodes[i]);
        BuildingsHelper::Install (ueNodes[i]);
    }
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (enbNodes);
    BuildingsHelper::Install (enbNodes);
    
    NetDeviceContainer enbDevs;
    NetDeviceContainer ueDevs[nEnb];
    
    enbDevs = lteHelper->InstallEnbDevice (enbNodes);
    for (i=0;i<nEnb;i++)
        ueDevs[i] = lteHelper->InstallUeDevice (ueNodes[i]);
    for (i=0;i<nEnb;i++)
        lteHelper->Attach (ueDevs[i], enbDevs.Get (i));
    
    enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
    EpsBearer bearer (q);
    for (i=0;i<nEnb;i++)
        lteHelper->ActivateDataRadioBearer (ueDevs[i], bearer);
    lteHelper->EnableTraces ();

    Simulator::Stop (Seconds (simTime));
    Simulator::Run ();
    Simulator::Destroy ();
    return 0;
}