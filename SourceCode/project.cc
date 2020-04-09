/* server
position of vehicle are obtained from socket and using that position of nodes are updated so that 
they can broadcast the vehicle position as wave short msg. Each node act as  wave devices.
 required header files are included in the file " priority_based_traffic_control.h " */

#include "priority_based_traffic_controll.h"
using namespace ns3;
int sockf, newsockf, port;
socklen_t clil;
char buf[256];
int n;
int flag=0;
struct BufferData
{
	std::string road,vehicle;
	double xpos,ypos;
	BufferData()
	{
		road=vehicle="";
	}
};
void dummyReply()
{
     	n = write(newsockf,"hii",3);
}
int checkposition(double xev,double yev,double xrsu,double yrsu,int vid){
	if(fabs(xev-xrsu)<=6.0 && fabs(yev-yrsu)<=6.0 && (vid%2!=0))
		return 1;
	else
		return 0;
}
// this method is to extract the road_id, x,y coordinate and vehicle id from the  buffer
BufferData parseData(char buffer[1024])
{

	BufferData result;
	std::string road="",vehicle="";
	std::string x="",y="";

	int k;
			for(k=0;buffer[k]!='@';k++)
			{
				road=road+buffer[k];
			}
			int j;
			for(j=k+1;buffer[j]!='$';j++)
			{
				x=x+buffer[j];
			}
	                int p;
			for(p=j+1;buffer[p]!='$';p++)
			{
				y=y+buffer[p];
			}
	        	for(int q=p+1;buffer[q]!='$';q++)
			{
	             		vehicle=vehicle+buffer[q];
	        	}

	result.road=road;
	result.vehicle=vehicle;
	result.xpos=(double)atof(x.c_str());
	result.ypos=(double)atof(y.c_str());
	return result;
}
// for getting current system time
const std::string currentDateTime()
 {
    time_t     now = time(0);
    struct tm  tstruct;
    char       bufr[80];
    tstruct = *localtime(&now);
    strftime(bufr, sizeof(bufr), "%Y-%m-%d.%X", &tstruct);

    return bufr;
}
void error(const char *msg)
{
    printf("%s\n",msg );
    exit(1);
}

class EV;
/* class   for road side unit */
class RSU
{
public :
		friend bool ReceivePacket (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
		void createRsuNodes();
		NodeContainer rsunodes;
		NetDeviceContainer rsudevices;
		friend RSU setupwave(RSU rsu,EV &ev);
};
/* creating required number of rsu in a fixed position */
void RSU::createRsuNodes ()
{

		rsunodes.Create (4);
  		Vector v1=Vector(400,493,0);
  		Vector v2=Vector(598,506,0);
  		Vector v3=Vector(493,598,0);
		Vector v4=Vector(506,400,0);
  		Ptr<ListPositionAllocator> pos=CreateObject<ListPositionAllocator>();
  	   	pos->Add(v1);
  		pos->Add(v2);
  	    	pos->Add(v3);
		pos->Add(v4);
  		MobilityHelper mob;
  		mob.SetPositionAllocator(pos);
  		mob.Install(rsunodes);
}
// making the server side ready to accept connection
void init()
{
    struct sockaddr_in serv_addr, cli_addr;
     sockf = socket(AF_INET, SOCK_STREAM, 0);
     if (sockf < 0) 
        error("ERROR opening socket");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     port=5555;
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(port);
     if (bind(sockf, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR on binding");
     listen(sockf,5);
     clil = sizeof(cli_addr);
     newsockf= accept(sockf, 
                 (struct sockaddr *) &cli_addr, 
                 &clil);
     if (newsockf < 0) 
          error("ERROR on accept");
}
// method to write data to socket when emergency vehicle arrived
void changeTraffic(std::string msg)
{
     
     bzero(buf,256);
     memcpy(buf, msg.c_str(),msg.size());
     n = write(newsockf,buf,strlen(buf));
}

/*function for rsu to receive packet */

bool ReceivePacket (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
	double rsux,rsuy;
	BufferData vehData;
	uint8_t buf[1024];
	memset(buf, 0, sizeof(buf));
	Ptr<Node> node = dev->GetNode();
	Ptr<MobilityModel> mobility= node->GetObject<MobilityModel> ();
	Vector pos = mobility->GetPosition();
	rsux=pos.x;
	rsuy=pos.y;
	if (pkt->GetSize () > 0)
	{
 	     pkt->CopyData(buf, 1024);

 	    char *bufdata = reinterpret_cast<char*>(buf);
 	     vehData= parseData(bufdata);
 	     std::cout<<"Position of Vehicle x "<<vehData.xpos<<"  y  "<<vehData.ypos<<"\n";
             int vid=atoi(vehData.vehicle.c_str());
 	     if(checkposition(round(vehData.xpos),round(vehData.ypos),rsux,rsuy,vid))
	     {
		 flag=1;
		 std::string msg=vehData.vehicle+"$"+vehData.road+"$";
                 changeTraffic(msg);
             }
 	     char t[10];
 	    sprintf(t,"%8.5f",Simulator::Now ().GetSeconds ());
 	    std::cout << t << " received a packet of "<< pkt->GetSize ()<< " byte : " <<buf<<" from :"<<sender<<" by "<<dev<<" at "<<currentDateTime()<<"\n" ;

	}

	return true;
}
/* class for  vehicles */
class EV
{
public :
		void createEvNodes();
		NodeContainer evnodes;
		NetDeviceContainer evdevices;
		friend void SendPacket (uint32_t channel,EV,Ptr<Node> node,Vector,std::string road);
		friend RSU setupwave(RSU rsu,EV &ev);
		bool ReceivePack(Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
};
/* creating required number of emergency vehicle nodes */
void EV::createEvNodes ()
{
		evnodes.Create (10);
		MobilityHelper mobility;
  		Vector v0=Vector(2401,1703,0);
  		Ptr<ListPositionAllocator> pos=CreateObject<ListPositionAllocator>();
  	        pos->Add(v0);
  		mobility.SetPositionAllocator(pos);
  		mobility.Install(evnodes);

}
/* implementation of wave protocol common for both EV and RSU class members */
RSU setupwave(RSU rsu,EV &ev)
{


  		YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default ();
  		YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  		wavePhy.SetChannel (waveChannel.Create ());
  		wavePhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  		QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  		WaveHelper waveHelper = WaveHelper::Default ();
  		ev.evdevices = waveHelper.Install (wavePhy, waveMac, ev.evnodes);
  		rsu.rsudevices = waveHelper.Install (wavePhy, waveMac, rsu.rsunodes);
  		// Tracing
  		wavePhy.EnablePcap ("EV", ev.evdevices);
  		wavePhy.EnablePcap ("RSU", rsu.rsudevices);
  		return rsu;
}
/* function for emergency vehicle to broadcast packet */
void SendPacket  (uint32_t channel,EV ev,Ptr<Node> node,Vector v,std::string roadnid)
{
		if(flag==0)
		{
			dummyReply();
                }
                flag=0;
		std::string id="";
		std::string road="";
		int k,j;
		for(k=0;roadnid[k]!='$';k++)
		{
			road=road+roadnid[k];
		}
		for(j=k+1;roadnid[j]!='$';j++)
		{
			id=id+roadnid[j];
		}
                int i=atoi(id.c_str());
	    	uint8_t  buf[1024];
		uint16_t bufsize;
		Ptr<MobilityModel> mobility= node->GetObject<MobilityModel> ();
		mobility->SetPosition(v);
        	AnimationInterface anim("jack.xml");
		std::string msg;
		std::stringstream ss;
		ss<<road;
		ss<<"@";
		ss<<v.x<<"$"<<v.y<<"$"<<i<<"$";
		msg=ss.str();
		bufsize = msg.size();
		memcpy(buf, msg.c_str(),bufsize);
		Ptr<Packet> p = Create<Packet> (buf, bufsize);
		Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (ev.evdevices.Get (i));
		const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
		Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
		const TxInfo txInfo = TxInfo (channel);
		sender->SendX  (p, bssWildcard, WSMP_PROT_NUMBER, txInfo);
		Simulator::Stop();
}



bool ReceivePack(Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
	uint8_t buf[1024];
	memset(buf, 0, sizeof(buf));
	if (pkt->GetSize () > 0)
	{
 	     pkt->CopyData(buf, 1024);
 	     char t[10];
 	    sprintf(t,"%8.5f",Simulator::Now ().GetSeconds ());
 	    std::cout << t << " received a packet of "<< pkt->GetSize ()<< " byte : " <<buf<<" from :"<<sender<<" by "<<dev<<" at "<<currentDateTime()<<"\n" ;

	}

	return true;
}


int main()
{
		EV ev;
		RSU rsu;
		ev.createEvNodes();
		rsu.createRsuNodes();
		rsu=setupwave(rsu,ev);
		Ptr<WaveNetDevice>  receiver1 = DynamicCast<WaveNetDevice> (rsu.rsudevices.Get (0));
		Ptr<WaveNetDevice>  receiver2 = DynamicCast<WaveNetDevice> (rsu.rsudevices.Get (1));
		Ptr<WaveNetDevice>  receiver3 = DynamicCast<WaveNetDevice> (rsu.rsudevices.Get (2));
		Ptr<WaveNetDevice>  receiver4 = DynamicCast<WaveNetDevice> (rsu.rsudevices.Get (3));
		Ptr<WaveNetDevice>  receiver11 = DynamicCast<WaveNetDevice> (ev.evdevices.Get (0));
		Ptr<WaveNetDevice>  receiver12 = DynamicCast<WaveNetDevice> (ev.evdevices.Get (2));
		Ptr<WaveNetDevice>  receiver13 = DynamicCast<WaveNetDevice> (ev.evdevices.Get (4));
		Ptr<WaveNetDevice>  receiver14 = DynamicCast<WaveNetDevice> (ev.evdevices.Get (6));
		Ptr<Node>node;
	int sockfd, newsockfd, portno = 55055,id;
    	socklen_t clilen;
    	char buffer[256];
    	struct sockaddr_in serv_addr, cli_addr;
    	int n;
    	sockfd = socket(AF_INET, SOCK_STREAM, 0);
    	if (sockfd < 0) 
       		error("ERROR opening socket");
    	bzero((char *) &serv_addr, sizeof(serv_addr));
    	serv_addr.sin_family = AF_INET;
    	serv_addr.sin_addr.s_addr = INADDR_ANY;
    	serv_addr.sin_port = htons(portno);
    	if (bind(sockfd, (struct sockaddr *) &serv_addr,
             sizeof(serv_addr)) < 0) 
        error("ERROR on binding");
    	if(listen(sockfd,5)!=0)
    		error("Error on listen");
    	clilen = sizeof(cli_addr);
    	newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    	if(newsockfd < 0)
    	error("ERROR on accept");
        init();
		receiver1->SetReceiveCallback(MakeCallback(&ReceivePacket));
		receiver2->SetReceiveCallback(MakeCallback(&ReceivePacket));
		receiver3->SetReceiveCallback(MakeCallback(&ReceivePacket));
		receiver4->SetReceiveCallback(MakeCallback(&ReceivePacket));
		receiver11->SetReceiveCallback(MakeCallback(&ReceivePack));
		receiver12->SetReceiveCallback(MakeCallback(&ReceivePack));
		receiver13->SetReceiveCallback(MakeCallback(&ReceivePack));
		receiver14->SetReceiveCallback(MakeCallback(&ReceivePack));
		std::string road,x,y,vehicle;
	for( int i=1;i<1400; i++)
   	{

		std::cout<<"Entered for loop i= "<<i<<" in main at "<<currentDateTime()<<"\n";
   		bzero(buffer, 255);
   		n=read(newsockfd, buffer, 255);
   		if(n<0)
   			error("ERROR ON READING FROM SOCKET");
		n = write(newsockfd,"I got your message",18);
		if (n < 0) error("ERROR writing to socket");
		BufferData sdata= parseData(buffer);
                id=atoi(sdata.vehicle.c_str());
                node=ev.evnodes.Get(id);
        	std::cout<<id;
		Vector pos;
		pos.x=sdata.xpos;
		pos.y=sdata.ypos;
		pos.z=0;
		std::string roadnid=sdata.road+"$"+sdata.vehicle+"$";
		std::cout<<roadnid<<"\n";
			Simulator::Schedule(Seconds(i),&SendPacket,CCH,ev,node,pos,roadnid);
			Simulator::Run();
   	}
    
	      AnimationInterface anim("jack.xml");
	      anim.UpdateNodeColor(rsu.rsunodes.Get(0),0,0,0);
	      anim.UpdateNodeColor(rsu.rsunodes.Get(1),0,0,0);
	      anim.UpdateNodeColor(rsu.rsunodes.Get(2),0,0,0);
	      Simulator::Stop (Seconds (10000.0));
	      Simulator::Destroy ();
	      return 0;
}
