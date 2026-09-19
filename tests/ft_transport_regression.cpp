// Standalone localhost transport regression; see tests/README.md.
#include "Y2FT_AQ/FT_EtherGet.hpp"
#include "Y2FT_AQ/FT_eCANGet.hpp"
#include <cassert>
#include <thread>
#include <limits>
#include <array>
#include <iostream>

int bindLocal(int kind, int& port) {
    int fd = socket(AF_INET, kind, 0); assert(fd >= 0);
    sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    assert(bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    socklen_t len=sizeof(addr); assert(getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len)==0);
    port=ntohs(addr.sin_port); return fd;
}
void settle() { std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
int main() {
    int port; int udp=bindLocal(SOCK_DGRAM,port);
    FT_EtherGet sensor("127.0.0.1",port);
    sockaddr_in peer{}; socklen_t plen=sizeof(peer); char request[32];
    assert(recvfrom(udp,request,sizeof(request),0,reinterpret_cast<sockaddr*>(&peer),&plen)>0);
    auto sendSample=[&](float x) {
        std::array<uint32_t,6> bytes{};
        for(auto& b:bytes) { std::memcpy(&b,&x,4); b=htonl(b); }
        assert(sendto(udp,bytes.data(),sizeof(bytes),0,reinterpret_cast<sockaddr*>(&peer),plen)==24);
        settle();
    };
    assert(!sensor.FTGet().fresh);
    sendSample(2.f); auto sample=sensor.FTGet(); assert(sample.fresh && sample.Fx==2.);
    assert(!sensor.FTGet().fresh);
    sendSample(std::numeric_limits<float>::quiet_NaN()); assert(!sensor.FTGet().fresh);
    sendSample(3.f); assert(sensor.FTGet().fresh);
    std::this_thread::sleep_for(std::chrono::milliseconds(5100));
    for(int i=0;i<5;++i) assert(!sensor.FT_init(2)); // No repeated stale samples in tare.
    sendSample(2.f); assert(!sensor.FT_init(2));
    for(int i=0;i<5;++i) assert(!sensor.FT_init(2));
    sendSample(4.f); sensor.FT_init(2); assert(sensor.FT_init(2));
    sendSample(5.f); sample=sensor.FTGet(); assert(sample.fresh && sample.Fx==2.);
    close(udp);

    int tcp=bindLocal(SOCK_STREAM,port); assert(listen(tcp,1)==0);
    FT_eCANGet can("127.0.0.1",port); int conn=accept(tcp,nullptr,nullptr); assert(conn>=0);
    assert(recv(conn,request,sizeof(request),0)>0);
    std::array<unsigned char,14> force{},moment{}; force[4]=1;moment[4]=2;
    for(int i=0;i<3;++i) {
        force[6+2*i]=30100>>8;force[7+2*i]=30100&255;
        moment[6+2*i]=25500>>8;moment[7+2*i]=25500&255;
    }
    assert(send(conn,force.data(),5,0)==5);settle(); assert(!can.FTGet().fresh);
    assert(send(conn,force.data()+5,9,0)==9);settle(); assert(!can.FTGet().fresh);
    assert(send(conn,moment.data(),7,0)==7);settle(); assert(!can.FTGet().fresh);
    assert(send(conn,moment.data()+7,7,0)==7);settle(); sample=can.FTGet();
    assert(sample.fresh && sample.Fx==1. && sample.Mx==1.);
    assert(!can.FTGet().fresh);
    // Coalesced frames must also be consumed without losing boundaries.
    std::array<unsigned char,28> pair{};
    std::copy(force.begin(),force.end(),pair.begin());std::copy(moment.begin(),moment.end(),pair.begin()+14);
    assert(send(conn,pair.data(),pair.size(),0)==28);settle();
    assert(!can.FTGet().fresh);assert(can.FTGet().fresh);
    close(conn);settle();assert(!can.FTGet().fresh);close(tcp);
    std::cout << "PASS: UDP freshness, NaN recovery, unique-sample tare, fragmented/coalesced TCP, disconnect\n";
}
