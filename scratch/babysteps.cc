#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/buildings-module.h"
#include "ns3/buildings-helper.h"
#include <vector>
#include <cmath>
#include <iostream>

using namespace ns3;

int main(int argc, char * argv[]){
    uint16_t nEnb = 10;
    uint16_t nUe = 30;
    uint16_t i = 0;
    uint16_t j = 0;
    double distance1 = 1000;
    double length = 10000;
    double distance2 = 30;
    double minSpeed = 1;
    double maxSpeed = 55;
    double minDelay = 0;
    double maxDelay = 20;
    uint16_t nBuildings = 12;
    double interPacketInterval = 100;
    double enbTxPowerDbm = 46.0;
    double simTime = 1;
    
    CommandLine cmd;
    cmd.AddValue("nUe", "Number of UEs", nUe);
    cmd.AddValue("nEnb", "Number of ENBs", nEnb);
    cmd.AddValue("distance1", "Distance between two successive ENBs in meters", distance1);
    cmd.AddValue("length", "Total length of line where the ENBs are located in meters", length);
    cmd.AddValue("distance2", "Distance between line of ENBs and line of UEs", distance2);
    cmd.AddValue("minSpeed", "Minimum possible speed of UEs in meters/second", minSpeed);
    cmd.AddValue("maxSpeed", "Maximum possible speed of UEs in meters/second", maxSpeed);
    cmd.AddValue("minDelay", "Minimum possible delay in seconds before a UE can begin traveling", minDelay);
    cmd.AddValue("maxDelay", "Maximum possible delay in seconds before a UE can begin traveling", maxDelay);
    cmd.AddValue("nBuildings", "Total number of building to be created", nBuildings);
    cmd.AddValue("interPacketInterval", "Inter packet interval in milliseconds", interPacketInterval);
    cmd.AddValue("simTime", "Duration of simulation in seconds", simTime);
    cmd.Parse(argc, argv);
    
    /*ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();
    cmd.Parse(argc, argv);*/
    
    Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (true));
    
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
    Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
    lteHelper->SetEpcHelper (epcHelper);
    
    lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");

    lteHelper->SetHandoverAlgorithmType ("ns3::A2A4RsrqHandoverAlgorithm");
    lteHelper->SetHandoverAlgorithmAttribute ("ServingCellThreshold",
                                            UintegerValue (30));
    lteHelper->SetHandoverAlgorithmAttribute ("NeighbourCellOffset",
                                            UintegerValue (1));
    
    Ptr<Node> pgw = epcHelper->GetPgwNode ();
    
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    InternetStackHelper internet;
    internet.Install (remoteHostContainer);

    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
    p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
    p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
    NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
    remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);
    
    NodeContainer enbNodes;
    NodeContainer ueNodes;
    enbNodes.Create (nEnb);
    ueNodes.Create (nUe);
    
    //Create the buildings
    Ptr<Building> buildings[nBuildings];
    for (i = 0; i < nBuildings; i++){
        buildings[i] = CreateObject<Building> ();
        buildings[i]->SetBoundaries (Box (100 * i, (100 * i + 50), 50, 100, 0, 12));
        buildings[i]->SetBuildingType (Building::Residential);
        buildings[i]->SetExtWallsType (Building::ConcreteWithWindows);
        buildings[i]->SetNFloors ((i % 6) + 1);
        buildings[i]->SetNRoomsX ((i % 3) + 1);
        buildings[i]->SetNRoomsY ((i % 3) + 1);
    }
    
    Ptr<HybridBuildingsPropagationLossModel> propagationLossModel = CreateObject<HybridBuildingsPropagationLossModel> ();
    
    
    Ptr<ListPositionAllocator> positionAllocEnb = CreateObject<ListPositionAllocator> ();
    for (i = 0; i < nEnb; i++)
        positionAllocEnb->Add (Vector(distance1 * i, 0, 0));
    MobilityHelper mobilityEnb;
    mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityEnb.SetPositionAllocator (positionAllocEnb);
    mobilityEnb.Install (enbNodes);
    
    //BuildingsHelper::MakeMobilityModelConsistent ();
    
    //Ptr<MobilityBuildingInfo> buildingInfoEnb = CreateObject<MobilityBuildingInfo> ();
    //mobilityEnb.AggregateObject (buildingInfoEnb);
    //BuildingsHelper::MakeMobilityModelConsistent ();
    
    MobilityHelper mobilityUe;
    Ptr<UniformRandomVariable> speed = CreateObject<UniformRandomVariable> ();
    speed->SetAttribute ("Min", DoubleValue (minSpeed));
    speed->SetAttribute ("Max", DoubleValue (maxSpeed));
    Ptr<UniformRandomVariable> pause = CreateObject<UniformRandomVariable> ();
    pause->SetAttribute ("Min", DoubleValue (minDelay));
    pause->SetAttribute ("Max", DoubleValue (maxDelay));
    Ptr<ListPositionAllocator> positionAllocUe = CreateObject<ListPositionAllocator> ();
    positionAllocUe->Add (Vector(0, distance2, 0));
    positionAllocUe->Add (Vector(length, distance2, 0));
    mobilityUe.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                               //"Speed", DoubleValue (speed->GetValue()),
                               //"Pause", RandomVariableValue (DoubleValue (pause->GetValue())),
                               "PositionAllocator", PointerValue (positionAllocUe));
    mobilityUe.SetPositionAllocator (positionAllocUe);
    //Ptr<MobilityBuildingInfo> buildingInfoUe = CreateObject<MobilityBuildingInfo> ();
    //mobilityUe.AggregateObject (buildingInfoUe); // operation usually done by BuildingsHelper::Install
    //BuildingsHelper::MakeMobilityModelConsistent ();
    mobilityUe.Install (ueNodes);
    //BuildingsHelper::MakeMobilityModelConsistent ();
    
    Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (enbTxPowerDbm));
    NetDeviceContainer enbDevs;
    NetDeviceContainer ueDevs;
    
    enbDevs = lteHelper->InstallEnbDevice (enbNodes);
    ueDevs = lteHelper->InstallUeDevice (ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    internet.Install (ueNodes);
    ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));
    
    // Install the IP stack on the UEs
    for (j = 0; j < ueNodes.GetN (); j++){
        Ptr<Node> ueNode = ueNodes.Get (j);
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
    
    // Attach the UEs to the eNodeBs
    lteHelper->Attach (ueDevs, enbDevs.Get (0));
    
    uint16_t dlPort = 1234;
    uint16_t ulPort = 2000;
    uint16_t otherPort = 3000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (j = 0; j < ueNodes.GetN (); j++){
            ++ulPort;
            ++otherPort;
            PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
            PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
            PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), otherPort));
            serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get(j)));
            serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
            serverApps.Add (packetSinkHelper.Install (ueNodes.Get(j)));

            UdpClientHelper dlClient (ueIpIface.GetAddress (j), dlPort);
            dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval)));
            dlClient.SetAttribute ("MaxPackets", UintegerValue(1000000));

            UdpClientHelper ulClient (remoteHostAddr, ulPort);
            ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval)));
            ulClient.SetAttribute ("MaxPackets", UintegerValue(1000000));

            UdpClientHelper client (ueIpIface.GetAddress (j), otherPort);
            client.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval)));
            client.SetAttribute ("MaxPackets", UintegerValue(1000000));

            clientApps.Add (dlClient.Install (remoteHost));
            clientApps.Add (ulClient.Install (ueNodes.Get(j)));
            if (j+1 < ceil (ueNodes.GetN ())){
                clientApps.Add (client.Install (ueNodes.Get(j+1)));
            }
            else{
            clientApps.Add (client.Install (ueNodes.Get(0)));
            }
    }
    serverApps.Start (Seconds (0.01));
    clientApps.Start (Seconds (0.02));
    
    lteHelper->AddX2Interface (enbNodes);
    
    Simulator::Stop (Seconds (simTime));
    Simulator::Run ();
    Simulator::Destroy ();
    return 0;
    
}