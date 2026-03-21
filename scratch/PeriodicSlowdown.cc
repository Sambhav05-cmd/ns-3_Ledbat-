#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LedbatPlusPlusSlowdown");

int main(int argc, char *argv[])
{
    LogComponentEnable("LedbatPlusPlusSlowdown", LOG_LEVEL_INFO);
    LogComponentEnable("TcpLedbatPlusPlus", LOG_LEVEL_INFO);

    NS_LOG_INFO("Simulation started");

    NodeContainer nodes;
    nodes.Create(2);        // Sender and receiver

    PointToPointHelper p2p;     // Set channel bandwidth and delay
    p2p.SetDeviceAttribute("DataRate", StringValue("1Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("50ms"));
    NetDeviceContainer devices = p2p.Install(nodes);

    InternetStackHelper internet;       // Install internet stack and its protocols onto the nodes
    internet.Install(nodes);

    TrafficControlHelper tch;
    tch.SetRootQueueDisc("ns3::PfifoFastQueueDisc",
                         "MaxSize", StringValue("20p"));    // 20 packets in the queue
    tch.Install(devices);

    Ipv4AddressHelper address;      // Set the ip addresses for the nodes 
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    Config::Set("/NodeList/0/$ns3::TcpL4Protocol/SocketType",
                TypeIdValue(TypeId::LookupByName("ns3::TcpLedbatPlusPlus")));       // Globally set the congestion protocol to LedbatPlusPlus

    PacketSinkHelper sink("ns3::TcpSocketFactory",      // Packet sink on node 1
        InetSocketAddress(Ipv4Address::GetAny(), 9000));
    sink.Install(nodes.Get(1)).Start(Seconds(0.0));

    BulkSendHelper sender("ns3::TcpSocketFactory",      // Packet sender on node 0
        InetSocketAddress(interfaces.GetAddress(1), 9000));
    sender.SetAttribute("MaxBytes", UintegerValue(0));
    sender.Install(nodes.Get(0)).Start(Seconds(0.5));

    Simulator::Stop(Seconds(120.0));        // Simulation runs for 120 seconds to see multiple slowdowns 
    Simulator::Run();
    Simulator::Destroy();

    NS_LOG_INFO("Simulation finished");
    return 0;
}