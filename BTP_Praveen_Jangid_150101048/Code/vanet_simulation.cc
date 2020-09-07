//Praveen Jangid 150101048
//B.Tech Thesis Project Work under the guidence of Dr. Moumita Patra Dept. of Computer Science IIT Guwahati 
//Simulation of .tcl mobility files to study the variation of probability factor (pcsmafactor in code) and number of nodes on performance of VANETs

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/itu-r-1411-los-propagation-loss-model.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/config-store-module.h"
#include "ns3/integer.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include <iostream>
#include <fstream>
using namespace ns3;
using namespace dsr;

NS_LOG_COMPONENT_DEFINE ("vanet-performance-compare"); //set up log file for tracing and debugging

void
vanetExperiment::ConfigureNodes ()  
{
	// set up nodes 
  m_adhocTxNodes.Create (m_nNodes);
}

void
vanetExperiment::ConfigureChannels ()
{
  // set up channel and devices
  SetupAdhocDevices ();
}

void
vanetExperiment::ConfigureDevices () 
{
  //set up trace for nodes
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/Tx", MakeCallback (&WifiPhyStats::PhyTxTrace, m_wifiPhyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback (&WifiPhyStats::PhyTxDrop, m_wifiPhyStats));
  Config::Connect ("/NodeList/*/DeviceList/*/ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback (&WifiPhyStats::PhyRxDrop, m_wifiPhyStats));
}

void
vanetExperiment::ConfigureMobility ()
{
  SetupAdhocMobilityNodes ();
}

void
vanetExperiment::ConfigureApplications ()
{

  SetupVanetMessages ();
  SetupWaveMessages ();

  // config trace to capture app-data (bytes) for
  // Vanet data, subtracted and used for
  // Vanet overhead
  std::ostringstream oss;
  oss.str ("");
  oss << "/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx";
  Config::Connect (oss.str (), MakeCallback (&VanetHelper::OnOffTrace, m_VanetHelper));
}

void
vanetExperiment::ConfigureTracing ()
{
	//Config Trace
  WriteCsvHeader ();
  SetupLogFile ();
  SetupLogging ();

  AsciiTraceHelper ascii;
  MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (m_trName + ".mob"));
}

class VanetStats  // to get data from pcap trace files
{
public:

  VanetStats ();

  uint32_t GetRxBytes ();
  uint32_t GetCumulativeRxBytes ();
  uint32_t GetRxPkts ();
  uint32_t GetCumulativeRxPkts ();
  	void IncRxBytes (uint32_t rxBytes);
  	void IncRxPkts ();
  	void SetRxBytes (uint32_t rxBytes);
  	void SetRxPkts (uint32_t rxPkts);
  uint32_t GetTxBytes ();
  uint32_t GetCumulativeTxBytes ();
  uint32_t GetTxPkts ();
  uint32_t GetCumulativeTxPkts ();
	void IncTxBytes (uint32_t txBytes);
 	void IncTxPkts ();
	void SetTxBytes (uint32_t txBytes);
 	void SetTxPkts (uint32_t txPkts);

private:
  uint32_t m_RxBytes; ///< reeive bytes
  uint32_t m_cumulativeRxBytes; ///< cumulative receive bytes
  uint32_t m_RxPkts; ///< receive packets
  uint32_t m_cumulativeRxPkts; ///< cumulative receive packets
  uint32_t m_TxBytes; ///< transmit bytes
  uint32_t m_cumulativeTxBytes; ///< cumulative transmit bytes
  uint32_t m_TxPkts; ///< transmit packets
  uint32_t m_cumulativeTxPkts; ///< cumulative transmit packets
};



void
VanetExperiment::SetupLogFile ()
{
  // open log file for output
  m_os.open (m_logFile.c_str ());
}

void VanetExperiment::SetupLogging ()
{

  // Enable logging from the ns2 helper
  LogComponentEnable ("Ns2MobilityHelper",LOG_LEVEL_DEBUG);

  Packet::EnablePrinting ();
}


void
VanetExperiment::SetupAdhocMobilityNodes () 
{
	//Set up mobility model for the mobility file 
      MobilityHelper mobilityAdhoc;

      ObjectFactory pos;
      pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
      pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));
      pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
      // we need antenna height uniform [1.0 .. 2.0] for loss model
      pos.Set ("Z", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"));

      Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
      m_streamIndex += taPositionAlloc->AssignStreams (m_streamIndex);

      std::stringstream ssSpeed;
      ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << m_nodeSpeed << "]";
      std::stringstream ssPause;
      ssPause << "ns3::ConstantRandomVariable[Constant=" << m_nodePause << "]";
      mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                      "Speed", StringValue (ssSpeed.str ()),
                                      "Pause", StringValue (ssPause.str ()),
                                      "PositionAllocator", PointerValue (taPositionAlloc));
      mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
      mobilityAdhoc.Install (m_adhocTxNodes);
      m_streamIndex += mobilityAdhoc.AssignStreams (m_adhocTxNodes, m_streamIndex);

      // initially assume all nodes are moving
      WaveBsmHelper::GetNodesMoving ().resize (m_nNodes, 1);
    

  // Configure callback for logging
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeBoundCallback (&VanetExperiment::CourseChange, &m_os));
}

void
VanetExperiment::SetupAdhocDevices () 
{
	//set up wave model in the node list of the mobility file 
  if (m_lossModel == 1)
    {
      m_lossModelName = "ns3::FriisPropagationLossModel";
    }
  else if (m_lossModel == 2)
    {
      m_lossModelName = "ns3::ItuR1411LosPropagationLossModel";
    }
  else if (m_lossModel == 3)
    {
      m_lossModelName = "ns3::TwoRayGroundPropagationLossModel";
    }
  else if (m_lossModel == 4)
    {
      m_lossModelName = "ns3::LogDistancePropagationLossModel";
    }
  else
    {
      // Unsupported propagation loss model.
      // Treating as ERROR
      NS_LOG_ERROR ("Invalid propagation loss model specified.  Values must be [1-4], where 1=Friis;2=ItuR1411Los;3=TwoRayGround;4=LogDistance");
    }

  // frequency
  double freq = 0.0;
  if ((m_80211mode == 1)
      || (m_80211mode == 3))
    {
      // 802.11p 5.9 GHz
      freq = 5.9e9;
    }
  else
    {
      // 802.11b 2.4 GHz
      freq = 2.4e9;
    }

  // Setup propagation models
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  if (m_lossModel == 3)
    {
      // two-ray requires antenna height (else defaults to Friss)
      wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq), "HeightAboveZ", DoubleValue (1.5));
    }
  else
    {
      wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq));
    }

  // Propagation loss models are additive.
  if (m_fading != 0)
    {
  
      wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
    }

  // the channel
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.SetChannel (channel);
  wavePhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

  // Setup WAVE PHY and MAC
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (m_verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
      // likewise, turn on WAVE PHY logging
      waveHelper.EnableLogComponents ();
    }

  WifiHelper wifi;

  // Setup 802.11b stuff
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (m_phyModeB),
                                "ControlMode",StringValue (m_phyModeB));

  // Setup 802.11p stuff
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (m_phyMode),
                                      "ControlMode",StringValue (m_phyMode));

  // Setup WAVE-PHY stuff
  waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (m_phyMode),
                                      "ControlMode",StringValue (m_phyMode));

  // Set Tx Power
  wifiPhy.Set ("TxPowerStart",DoubleValue (m_txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txp));
  wavePhy.Set ("TxPowerStart",DoubleValue (m_txp));
  wavePhy.Set ("TxPowerEnd", DoubleValue (m_txp));

  // Add an upper mac and disable rate control
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();

  // Setup net devices

  if (m_80211mode == 3)
    {
      m_adhocTxDevices = waveHelper.Install (wavePhy, waveMac, m_adhocTxNodes);
    }
  else if (m_80211mode == 1)
    {
      m_adhocTxDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, m_adhocTxNodes);
    }
  else
    {
      m_adhocTxDevices = wifi.Install (wifiPhy, wifiMac, m_adhocTxNodes);
    }

  if (m_asciiTrace != 0)
    {
      AsciiTraceHelper ascii;
      Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (m_trName + ".tr").c_str ());
      wifiPhy.EnableAsciiAll (osw);
      wavePhy.EnableAsciiAll (osw);
    }
  if (m_pcap != 0)
    {
      wifiPhy.EnablePcapAll ("vanet--compare-pcap");
      wavePhy.EnablePcapAll ("vanet--compare-pcap");
    }
}

void
VanetExperiment::SetupWaveMessages ()
{
  // WAVE PHY mode
  // 0=continuous channel; 1=channel-switching
  int chAccessMode = 0;
  if (m_80211mode == 3)
    {
      chAccessMode = 1;
    }

  m_waveBsmHelper.Install (m_adhocTxInterfaces,
                           Seconds (m_TotalSimTime),
                           m_wavePacketSize,
                           Seconds (m_waveInterval),
                           // GPS accuracy (i.e, clock drift), in number of ns
                           m_gpsAccuracyNs,
                           m_txSafetyRanges,
                           chAccessMode,
                           // tx max delay before transmit, in ms
                           MilliSeconds (m_txMaxDelayMs));

  // fix random number streams
  m_streamIndex += m_waveBsmHelper.AssignStreams (m_adhocTxNodes, m_streamIndex);
}

void
VanetExperiment::SetupMessages ()
{
  m_Helper->Install (m_adhocTxNodes,
                            m_adhocTxDevices,
                            m_adhocTxInterfaces,
                            m_TotalSimTime,
                            m_protocol,
                            m_nSinks,
                            m_Tables);
}

void
VanetExperiment::SetupScenario ()
{
  // Setting up model scenario using the trace file 
      m_traceFile = "/home/praveen/mobility.tcl";  // add your custom trace file here AddMobilityFile
      m_logFile = "mobpraveen.log";
      m_mobility = 1;
      m_nMaxNodes = 50;
      m_TotalSimTime = 10.01;
      m_nodeSpeed = 20;
      m_nodePause = 0;
      m_CSVfileName = "mobpraveen.csv";
      m_CSVfileName = "mobpraveen2.csv";
}
void
vanetHelper::SetupMessages (NodeContainer & c,
                                     Ipv4InterfaceContainer & adhocTxInterfaces)
{
	//probability conditioning on the transmission function 
	float pcsmafactor=.3;    //change probability factor here 
    srand( (unsigned)time( NULL ) );
    float probability=(float) rand()/RAND_MAX << endl;
    if(probability>pcsmafactor)
    {
    	  // Setup  transmissions
  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
  int64_t stream = 2;
  var->SetStream (stream);
  for (uint32_t i = 0; i < m_nSinks; i++)
    {
      // protocol == 0 means no  data, WAVE BSM only
      // so do not set up sink
      if (m_protocol != 0)
        {
          Ptr<Socket> sink = SetupPacketReceive (adhocTxInterfaces.GetAddress (i), c.Get (i));
        }

      AddressValue remoteAddress (InetSocketAddress (adhocTxInterfaces.GetAddress (i), m_port));
      onoff1.SetAttribute ("Remote", remoteAddress);

      ApplicationContainer temp = onoff1.Install (c.Get (i + m_nSinks));
      temp.Start (Seconds (var->GetValue (1.0,2.0)));
      temp.Stop (Seconds (m_TotalSimTime));
    }
    }
}

void
VanetExperiment::WriteCsvHeader ()
{
  //blank out the last output file and write the column headers
  std::ofstream out (m_CSVfileName.c_str ());
  out << "SimulationSecond," <<
    "ReceiveRate," <<
    "PacketsReceived," <<
    "NumberOfSinks," <<
    "Protocol," <<
    "TransmissionPower," <<
    "WavePktsSent," <<
    "WavePtksReceived," <<
    "WavePktsPpr," <<
    "ExpectedWavePktsReceived," <<
    "ExpectedWavePktsInCoverageReceived," <<
    "BSM_PDR1," <<
    "BSM_PDR2," <<
    "BSM_PDR3," <<
    "BSM_PDR4," <<
    "BSM_PDR5," <<
    "BSM_PDR6," <<
    "BSM_PDR7," <<
    "BSM_PDR8," <<
    "BSM_PDR9," <<
    "BSM_PDR10," <<
    "MacPhyOverhead" <<
    std::endl;
  out.close ();

  std::ofstream out2 (m_CSVfileName2.c_str ());
  out2 << "BSM_PDR1,"
       << "BSM_PDR2,"
       << "BSM_PDR3,"
       << "BSM_PDR4,"
       << "BSM_PDR5,"
       << "BSM_PDR6,"
       << "BSM_PDR7,"
       << "BSM_PDR8,"
       << "BSM_PDR9,"
       << "BSM_PDR10,"
       << "AverageGoodputKbps,"
       << "MacPhyOverhead"
       << std::endl;
  out2.close ();
}
void
vanetExperiment::Run ()
{
  NS_LOG_INFO ("Run Simulation.");

  CheckThroughput ();

  Simulator::Stop (Seconds (m_TotalSimTime));
  Simulator::Run ();
  Simulator::Destroy ();
}
void
vanetExperiment::CheckThroughput ()
{
  uint32_t bytesTotal = m_VanetHelper->GetVanetStats ().GetRxBytes ();
  uint32_t packetsReceived = m_VanetHelper->GetVanetStats ().GetRxPkts ();
  double kbps = (bytesTotal * 8.0) / 1000;
  double wavePDR = 0.0;
  int wavePktsSent = m_waveBsmHelper.GetWaveBsmStats ()->GetTxPktCount ();
  int wavePktsReceived = m_waveBsmHelper.GetWaveBsmStats ()->GetRxPktCount ();
  if (wavePktsSent > 0)
    {
      int wavePktsReceived = m_waveBsmHelper.GetWaveBsmStats ()->GetRxPktCount ();
      wavePDR = (double) wavePktsReceived / (double) wavePktsSent;
    }

  int waveExpectedRxPktCount = m_waveBsmHelper.GetWaveBsmStats ()->GetExpectedRxPktCount (1);
  int waveRxPktInRangeCount = m_waveBsmHelper.GetWaveBsmStats ()->GetRxPktInRangeCount (1);
  double wavePDR1_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (1);
  double wavePDR2_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (2);
  double wavePDR3_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (3);
  double wavePDR4_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (4);
  double wavePDR5_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (5);
  double wavePDR6_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (6);
  double wavePDR7_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (7);
  double wavePDR8_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (8);
  double wavePDR9_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (9);
  double wavePDR10_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (10);

  // calculate MAC/PHY overhead (mac-phy-oh)
  // total WAVE BSM bytes sent
  uint32_t cumulativeWaveBsmBytes = m_waveBsmHelper.GetWaveBsmStats ()->GetTxByteCount ();
  uint32_t cumulativeVanetBytes = m_VanetHelper->GetVanetStats ().GetCumulativeTxBytes ();
  uint32_t totalAppBytes = cumulativeWaveBsmBytes + cumulativeVanetBytes;
  uint32_t totalPhyBytes = m_wifiPhyStats->GetTxBytes ();
  // mac-phy-oh = (total-phy-bytes - total-app-bytes) / total-phy-bytes
  double mac_phy_oh = 0.0;
  if (totalPhyBytes > 0)
    {
      mac_phy_oh = (double) (totalPhyBytes - totalAppBytes) / (double) totalPhyBytes;
    }

  std::ofstream out (m_CSVfileName.c_str (), std::ios::app);

  if (m_log != 0 )
    {
      NS_LOG_UNCOND ("At t=" << (Simulator::Now ()).GetSeconds () << "s BSM_PDR1=" << wavePDR1_2 << " BSM_PDR1=" << wavePDR2_2 << " BSM_PDR3=" << wavePDR3_2 << " BSM_PDR4=" << wavePDR4_2 << " BSM_PDR5=" << wavePDR5_2 << " BSM_PDR6=" << wavePDR6_2 << " BSM_PDR7=" << wavePDR7_2 << " BSM_PDR8=" << wavePDR8_2 << " BSM_PDR9=" << wavePDR9_2 << " BSM_PDR10=" << wavePDR10_2 << " Goodput=" << kbps << "Kbps" /*<< " MAC/PHY-OH=" << mac_phy_oh*/);
    }

  out << (Simulator::Now ()).GetSeconds () << ","
      << kbps << ","
      << packetsReceived << ","
      << m_nSinks << ","
      << m_protocolName << ","
      << m_txp << ","
      << wavePktsSent << ","
      << wavePktsReceived << ","
      << wavePDR << ","
      << waveExpectedRxPktCount << ","
      << waveRxPktInRangeCount << ","
      << wavePDR1_2 << ","
      << wavePDR2_2 << ","
      << wavePDR3_2 << ","
      << wavePDR4_2 << ","
      << wavePDR5_2 << ","
      << wavePDR6_2 << ","
      << wavePDR7_2 << ","
      << wavePDR8_2 << ","
      << wavePDR9_2 << ","
      << wavePDR10_2 << ","
      << mac_phy_oh << ""
      << std::endl;

  out.close ();

  m_VanetHelper->GetVanetStats ().SetRxBytes (0);
  m_VanetHelper->GetVanetStats ().SetRxPkts (0);
  m_waveBsmHelper.GetWaveBsmStats ()->SetRxPktCount (0);
  m_waveBsmHelper.GetWaveBsmStats ()->SetTxPktCount (0);
  for (int index = 1; index <= 10; index++)
    {
      m_waveBsmHelper.GetWaveBsmStats ()->SetExpectedRxPktCount (index, 0);
      m_waveBsmHelper.GetWaveBsmStats ()->SetRxPktInRangeCount (index, 0);
    }

  double currentTime = (Simulator::Now ()).GetSeconds ();
  if (currentTime <= (double) m_cumulativeBsmCaptureStart)
    {
      for (int index = 1; index <= 10; index++)
        {
          m_waveBsmHelper.GetWaveBsmStats ()->ResetTotalRxPktCounts (index);
        }
    }

  Simulator::Schedule (Seconds (1.0), &vanetExperiment::CheckThroughput, this);
}

void
vanetExperiment::ProcessOutputs ()
{
  // calculate and output final results
  double bsm_pdr1 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (1);
  double bsm_pdr2 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (2);
  double bsm_pdr3 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (3);
  double bsm_pdr4 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (4);
  double bsm_pdr5 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (5);
  double bsm_pdr6 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (6);
  double bsm_pdr7 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (7);
  double bsm_pdr8 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (8);
  double bsm_pdr9 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (9);
  double bsm_pdr10 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (10);

  double averageVanetgoodputKbps = 0.0;
  uint32_t totalBytesTotal = m_VanetHelper->GetVanetStats ().GetCumulativeRxBytes ();
  averageVanetgoodputKbps = (((double) totalBytesTotal * 8.0) / m_TotalSimTime) / 1000.0;

  // calculate MAC/PHY overhead (mac-phy-oh)
  // total WAVE BSM bytes sent
  uint32_t cumulativeWaveBsmBytes = m_waveBsmHelper.GetWaveBsmStats ()->GetTxByteCount ();
  uint32_t cumulativeVanetBytes = m_VanetHelper->GetVanetStats ().GetCumulativeTxBytes ();
  uint32_t totalAppBytes = cumulativeWaveBsmBytes + cumulativeVanetBytes;
  uint32_t totalPhyBytes = m_wifiPhyStats->GetTxBytes ();
  // mac-phy-oh = (total-phy-bytes - total-app-bytes) / total-phy-bytes
  double mac_phy_oh = 0.0;
  if (totalPhyBytes > 0)
    {
      mac_phy_oh = (double) (totalPhyBytes - totalAppBytes) / (double) totalPhyBytes;
    }

  if (m_log != 0)
    {
      NS_LOG_UNCOND ("BSM_PDR1=" << bsm_pdr1 << " BSM_PDR2=" << bsm_pdr2 << " BSM_PDR3=" << bsm_pdr3 << " BSM_PDR4=" << bsm_pdr4 << " BSM_PDR5=" << bsm_pdr5 << " BSM_PDR6=" << bsm_pdr6 << " BSM_PDR7=" << bsm_pdr7 << " BSM_PDR8=" << bsm_pdr8 << " BSM_PDR9=" << bsm_pdr9 << " BSM_PDR10=" << bsm_pdr10 << " Goodput=" << averageVanetgoodputKbps << "Kbps MAC/PHY-oh=" << mac_phy_oh);

    }

  std::ofstream out (m_CSVfileName2.c_str (), std::ios::app);

  out << bsm_pdr1 << ","
      << bsm_pdr2 << ","
      << bsm_pdr3 << ","
      << bsm_pdr4 << ","
      << bsm_pdr5 << ","
      << bsm_pdr6 << ","
      << bsm_pdr7 << ","
      << bsm_pdr8 << ","
      << bsm_pdr9 << ","
      << bsm_pdr10 << ","
      << averageVanetgoodputKbps << ","
      << mac_phy_oh << ""
      << std::endl;

  out.close ();

  m_os.close (); // close log file
}


int main (int argc, char *argv[])
{

  bool verbose = false;   //setting up verbose flag as false

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
    }

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, c);

  // Tracing
  wifiPhy.EnablePcap ("wave-simple-80211p", devices);

  



  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
