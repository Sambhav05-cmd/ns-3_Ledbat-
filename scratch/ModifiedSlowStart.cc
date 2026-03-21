#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LedbatPlusPlusModifiedSlowStart");

int main(int argc, char* argv[])
{
    LogComponentEnable("LedbatPlusPlusModifiedSlowStart", LOG_LEVEL_ALL);
    LogComponentEnable("TcpLedbatPlusPlus", LOG_LEVEL_INFO);

    NS_LOG_INFO("Simulation Started");

    NodeContainer senders;
    senders.Create(2);

    NodeContainer receiver;
    receiver.Create(1);

    NodeContainer router;
    router.Create(1);

    PointToPointHelper link;
    link.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    link.SetChannelAttribute("Delay", StringValue("1ms"));

    NetDeviceContainer n0 = link.Install(NodeContainer(senders.Get(0), router.Get(0)));
    NetDeviceContainer n1 = link.Install(NodeContainer(senders.Get(1), router.Get(0)));

    PointToPointHelper bottleneckLink;
    bottleneckLink.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    bottleneckLink.SetChannelAttribute("Delay", StringValue("50ms"));
    
    NetDeviceContainer n2 = bottleneckLink.Install(NodeContainer(router.Get(0), receiver.Get(0)));

    InternetStackHelper internet;
    internet.Install(senders);
    internet.Install(router);
    internet.Install(receiver);

    TrafficControlHelper tch;
    tch.SetRootQueueDisc("ns3::PfifoFastQueueDisc", "MaxSize", StringValue("10p"));
    tch.Install(n2);

    Ipv4AddressHelper address;

    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i0 = address.Assign(n0);

    address.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer i1 = address.Assign(n1);

    address.SetBase("10.1.3.0", "255.255.255.0");
    Ipv4InterfaceContainer i2 = address.Assign(n2);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // sink on receiver for both flows
    PacketSinkHelper cubicSink("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 6000));
    cubicSink.Install(receiver.Get(0)).Start(Seconds(0.0));

    PacketSinkHelper ledbatSink("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 7000));
    ledbatSink.Install(receiver.Get(0)).Start(Seconds(0.0));

    // Cubic sender on node 0
    Config::Set("/NodeList/0/$ns3::TcpL4Protocol/SocketType", TypeIdValue(TypeId::LookupByName("ns3::TcpCubic")));

    BulkSendHelper cubicSender("ns3::TcpSocketFactory", InetSocketAddress(i2.GetAddress(1), 6000));
    cubicSender.SetAttribute("MaxBytes", UintegerValue(0));
    cubicSender.Install(senders.Get(0)).Start(Seconds(5.0));

    // LEDBAT++ sender on node 1
    Config::Set("/NodeList/1/$ns3::TcpL4Protocol/SocketType", TypeIdValue(TypeId::LookupByName("ns3::TcpLedbatPlusPlus")));

    BulkSendHelper ledbatSender("ns3::TcpSocketFactory", InetSocketAddress(i2.GetAddress(1), 7000));
    ledbatSender.SetAttribute("MaxBytes", UintegerValue(0));
    ledbatSender.Install(senders.Get(1)).Start(Seconds(0.5));

    Simulator::Stop(Seconds(30.0));
    Simulator::Run();
    Simulator::Destroy();

    NS_LOG_INFO("Simulation finished");
    return 0;


}