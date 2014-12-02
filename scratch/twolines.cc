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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TwoLinesLog");

int main(int argc, char* argv[]){
    
    uint16_t nEnb = 10;
    uint16_t nUe = 50;
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
    double simTime = 10;
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
    cmd.AddValue("interPacketInterval", "Inter packet interval in miliseconds", interPacketInterval);
    cmd.AddValue("simTime", "Duration of simulation in seconds", simTime);
    cmd.Parse(argc, argv);
    
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
    Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
    lteHelper->SetEpcHelper (epcHelper);

    //ConfigStore inputConfig;
    //inputConfig.ConfigureDefaults();
    //cmd.Parse(argc, argv);

    Ptr<Node> pgw = epcHelper->GetPgwNode ();

    // Create a single RemoteHost
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
    NodeContainer ueNodes[nEnb];
    enbNodes.Create (nEnb);
    for (i = 0; i < nEnb-1; i++)
        ueNodes[i].Create (nUe/nEnb);
    ueNodes[nEnb-1].Create (nUe-((nUe/nEnb)*(nEnb-1)));
    
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
    
    //Ptr<HybridBuildingsPropagationLossModel> propagationLossModel = CreateObject<HybridBuildingsPropagationLossModel> ();
    
    // Install Mobility Models
    Ptr<ListPositionAllocator> positionAllocEnb = CreateObject<ListPositionAllocator> ();
    for (i = 0; i < nEnb; i++)
        positionAllocEnb->Add (Vector(distance1 * i, 0, 0));
    MobilityHelper mobilityEnb;
    mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityEnb.SetPositionAllocator (positionAllocEnb);
    mobilityEnb.Install (enbNodes);
    
    //Ptr<MobilityBuildingInfo> buildingInfoEnb = CreateObject<MobilityBuildingInfo> ();
    //mobilityEnb->AggregateObject (buildingInfoEnb);
    //BuildingsHelper::MakeConsistent (mobilityEnb);
    
    MobilityHelper mobilityUe;
    Ptr<ListPositionAllocator> positionAllocUe = CreateObject<ListPositionAllocator> ();
    positionAllocUe->Add (Vector(0, distance2, 0));
    positionAllocUe->Add (Vector(length, distance2, 0));
    mobilityUe.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                               //"Speed", RandomVariableValue (UniformVariable (minSpeed, maxSpeed)),
                               //"Pause", RandomVariableValue (UniformVariable (minDelay, maxDelay)),
                               "PositionAllocator", PointerValue (positionAllocUe));
    mobilityUe.SetPositionAllocator (positionAllocUe);
    //Ptr<MobilityBuildingInfo> buildingInfoUe = CreateObject<MobilityBuildingInfo> ();
    //mobilityUe->AggregateObject (buildingInfoUe); // operation usually done by BuildingsHelper::Install
    //BuildingsHelper::MakeConsistent (mobilityUe);
    for (i=0; i < nEnb; i++){
        mobilityUe.Install (ueNodes[i]);
    }
    
    // Install LTE Devices to the nodes
    NetDeviceContainer enbDevs;
    NetDeviceContainer ueDevs[nEnb];
    
    enbDevs = lteHelper->InstallEnbDevice (enbNodes);
    for (i = 0; i < nEnb; i++)
        ueDevs[i] = lteHelper->InstallUeDevice (ueNodes[i]);
    Ipv4InterfaceContainer ueIpIface[nEnb];
    for (i=0; i < nEnb; i++){
        internet.Install (ueNodes[i]);
        ueIpIface[i] = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs[i]));
    }
    
    // Install the IP stack on the UEs
    for (i = 0; i < nEnb; i++)
        for (j = 0; j < ueNodes[i].GetN (); j++){
        Ptr<Node> ueNode = ueNodes[i].Get (j);
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
    
    // Attach the UEs to the eNodeBs
    for (i = 0; i < nEnb; i++)
        lteHelper->Attach (ueDevs[i], enbDevs.Get (i));
    
    // Install and start applications on UEs and remote host
    uint16_t dlPort = 1234;
    uint16_t ulPort = 2000;
    uint16_t otherPort = 3000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (i = 0; i < nEnb; i++)
        for (j = 0; j < ueNodes[i].GetN (); j++){
            ++ulPort;
            ++otherPort;
            PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
            PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
            PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), otherPort));
            serverApps.Add (dlPacketSinkHelper.Install (ueNodes[i].Get(j)));
            serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
            serverApps.Add (packetSinkHelper.Install (ueNodes[i].Get(j)));

            UdpClientHelper dlClient (ueIpIface[i].GetAddress (j), dlPort);
            dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval)));
            dlClient.SetAttribute ("MaxPackets", UintegerValue(1000000));

            UdpClientHelper ulClient (remoteHostAddr, ulPort);
            ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval)));
            ulClient.SetAttribute ("MaxPackets", UintegerValue(1000000));

            UdpClientHelper client (ueIpIface[i].GetAddress (j), otherPort);
            client.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval)));
            client.SetAttribute ("MaxPackets", UintegerValue(1000000));

            clientApps.Add (dlClient.Install (remoteHost));
            clientApps.Add (ulClient.Install (ueNodes[i].Get(j)));
            if (j+1 < ceil (ueNodes[i].GetN ())){
                clientApps.Add (client.Install (ueNodes[i].Get(j+1)));
            }
            else{
            clientApps.Add (client.Install (ueNodes[i].Get(0)));
            }
    }
    serverApps.Start (Seconds (0.01));
    clientApps.Start (Seconds (0.01));
    
    Simulator::Stop (Seconds (simTime));
    Simulator::Run ();
    Simulator::Destroy ();
    return 0;
}